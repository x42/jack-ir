/* jack-ir - JACK Impulse Response Capture Utility
 *
 * Copyright (C) 2019 Robin Gareus <robin@gareus.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef _WIN32
#include <signal.h>
#endif

#include <algorithm>
#include <string>
#include <vector>

#include <jack/jack.h>
#include <sndfile.h>

#include "zita-convolver.h"

using namespace IrJackZitaConvolver;

static uint32_t n_ir      = 0;
static uint32_t n_inputs  = 2;
static uint32_t n_outputs = 2;

static bool     true_stereo      = false;
static uint32_t true_stereo_pass = 1;

static float** ir        = NULL;
static float*  sweep_sin = NULL;
static float*  sweep_inv = NULL;

static uint32_t sweep_len = 0;
static uint32_t irrec_len = 0;

static uint32_t proc_pos = 0;
static uint32_t proc_tot = 0;

static jack_port_t** output_ports = NULL;
static jack_port_t** input_ports  = NULL;

static uint32_t roundtrip_latency = 0;

static volatile enum {
	Initialize,
	Run,
	Exit,
	Abort
} client_state = Initialize;

static void
process_multi_pass (jack_nframes_t n_samples)
{
	assert (n_outputs == 2 && n_inputs == 2 && n_ir == 4);
	bool fp = true_stereo_pass > 0;

	if (proc_pos < sweep_len) {
		uint32_t n_play = proc_pos + n_samples < sweep_len ? n_samples : sweep_len - proc_pos;
		float*   out    = (float*)jack_port_get_buffer (output_ports[fp ? 0 : 1], n_samples);
		memcpy (out, &sweep_sin[proc_pos], n_play * sizeof (float));
	}

	if (proc_pos < irrec_len) {
		uint32_t n_rec = proc_pos + n_samples < irrec_len ? n_samples : irrec_len - proc_pos;
		for (uint32_t n = 0; n < 2; ++n) {
			float* in = (float*)jack_port_get_buffer (input_ports[n], n_samples);
			memcpy (&ir[n + (fp ? 0 : 2)][proc_pos], in, n_rec * sizeof (float));
		}
	}

	proc_pos += n_samples;

	if (proc_pos > irrec_len + true_stereo_pass) {
		if (fp) {
			proc_pos         = 0;
			true_stereo_pass = 0;
		} else {
			client_state = Exit;
		}
	}
}

static void
process_single_pass (jack_nframes_t n_samples)
{
	assert (n_inputs == n_ir);

	if (proc_pos < sweep_len) {
		uint32_t n_play = proc_pos + n_samples < sweep_len ? n_samples : sweep_len - proc_pos;
		for (uint32_t n = 0; n < n_outputs; ++n) {
			float* out = (float*)jack_port_get_buffer (output_ports[n], n_samples);
			memcpy (out, &sweep_sin[proc_pos], n_play * sizeof (float));
		}
	}

	if (proc_pos < irrec_len) {
		uint32_t n_rec = proc_pos + n_samples < irrec_len ? n_samples : irrec_len - proc_pos;
		for (uint32_t n = 0; n < n_inputs; ++n) {
			float* in = (float*)jack_port_get_buffer (input_ports[n], n_samples);
			memcpy (&ir[n][proc_pos], in, n_rec * sizeof (float));
		}
	}

	proc_pos += n_samples;

	if (proc_pos > irrec_len) {
		client_state = Exit;
	}
}

static int
jack_process (jack_nframes_t n_samples, void* arg)
{
	for (uint32_t n = 0; n < n_outputs; ++n) {
		float* out = (float*)jack_port_get_buffer (output_ports[n], n_samples);
		memset (out, 0, sizeof (float) * n_samples);
	}

	if (client_state != Run) {
		return 0;
	}

	if (true_stereo) {
		process_multi_pass (n_samples);
	} else {
		process_single_pass (n_samples);
	}

	proc_tot += n_samples;
	return 0;
}

static int
jack_xrun (void* arg)
{
	fprintf (stderr, "JACK x-run, aborting\n");
	client_state = Abort;
	return 0;
}

static void
jack_shutdown (void* arg)
{
	fprintf (stderr, "JACK terminated, aborting\n");
	client_state = Abort;
}

static int
jack_graph_order_cb (void* arg)
{
	uint32_t worst_capture  = 0;
	uint32_t worst_playback = 0;
	for (uint32_t n = 0; n < n_inputs; ++n) {
		jack_latency_range_t lr;
		jack_port_get_latency_range (input_ports[n], JackCaptureLatency, &lr);
		if (lr.max > worst_capture) {
			worst_capture = lr.max;
		}
	}
	for (uint32_t n = 0; n < n_outputs; ++n) {
		jack_latency_range_t lr;
		jack_port_get_latency_range (output_ports[n], JackPlaybackLatency, &lr);
		if (lr.max > worst_playback) {
			worst_playback = lr.max;
		}
	}
	roundtrip_latency = worst_capture + worst_playback;
	return 0;
}

static int
sf_write (const char* fn, uint32_t n_channels, uint32_t rate, uint32_t off_start, uint32_t n_frames, float** data)
{
	SNDFILE* file;
	SF_INFO  sfinfo;

	memset (&sfinfo, 0, sizeof (sfinfo));

	if (n_channels > 4) {
		return -1;
	}

	sfinfo.samplerate = rate;
	sfinfo.frames     = n_frames;
	sfinfo.channels   = n_channels;
	sfinfo.format     = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

	if (!(file = sf_open (fn, SFM_WRITE, &sfinfo))) {
		fprintf (stderr, "Error: Not able to open output file '%s'.\n", fn);
		return -1;
	}

	for (uint32_t f = off_start; f < off_start + n_frames; ++f) {
		float frame[4];
		for (uint32_t c = 0; c < n_channels; ++c) {
			frame[c] = data[c][f];
		}
		if (1 != sf_writef_float (file, frame, 1)) {
			fprintf (stderr, "Error wrting file '%s': %s\n", fn, sf_strerror (file));
			sf_close (file);
			return -2;
		}
	}

	sf_close (file);
	return 0;
}

static int
convolv (uint32_t n_channels, uint32_t n_samples, float** data)
{
	Convproc p;

	int rv = p.configure (
	    /* in */ n_channels,
	    /* out */ n_channels,
	    /* max-convolution length */ sweep_len,
	    /* quantum, nominal-buffersize */ Convproc::MAXPART,
	    /* Convproc::MINPART */ Convproc::MAXPART,
	    /* Convproc::MAXPART */ Convproc::MAXPART,
	    /* density */ 0);

	if (rv != 0) {
		return rv;
	}

	rv = p.impdata_create (
	    /*i/o map */ 0, 0,
	    /*stride, de-interleave */ 1,
	    sweep_inv,
	    0, sweep_len);

	if (rv != 0) {
		return rv;
	}

	for (uint32_t c = 1; c < n_channels; ++c) {
		if (p.impdata_link (0, 0, c, c)) {
			return -1;
		}
	}

	if (p.start_process (0, 0)) {
		return -1;
	}

	if (p.state () != Convproc::ST_PROC) {
		return -1;
	}

	uint32_t off      = 0;
	uint32_t n_remain = n_samples;

	while (n_remain > 0) {
		uint32_t n = std::min (n_remain, (uint32_t)Convproc::MAXPART);

		for (uint32_t c = 0; c < n_channels; ++c) {
			float* const in = p.inpdata (c);
			if (n < Convproc::MAXPART) {
				memset (in, 0, sizeof (float) * Convproc::MAXPART);
			}
			memcpy (in, &data[c][off], sizeof (float) * n);
		}

		p.process ();

		for (uint32_t c = 0; c < n_channels; ++c) {
			float const* const out = p.outdata (c);
			memcpy (&data[c][off], out, sizeof (float) * n);
		}

		n_remain -= n;
		off += n;
	}
	return 0;
}

static uint32_t
trim_end (uint32_t n_channels, uint32_t rate, uint32_t n_samples, float** data)
{
	float    sig_lvl = exp10f (.05 * -20);
	float    sig_min = exp10f (.05 * -60);
	uint32_t tme_min = rate / 20;

	assert (n_samples > tme_min);

	uint32_t tme_trim = n_samples;
	uint32_t t        = 0;
	bool     init     = true;

	for (uint32_t n = 0; n < n_samples; ++n) {
		bool silent = !init;
		for (uint32_t c = 0; c < n_channels; ++c) {
			float s = fabsf (data[c][n]);
			if (s > sig_lvl) {
				init = false;
			}
			if (s > sig_min) {
				silent = false;
			}
		}
		if (silent) {
			if (++t > tme_min) {
				tme_trim = n;
				break;
			}
		} else {
			t = 0;
		}
	}

	assert (tme_trim >= tme_min);

	/* fade-out tail */
	uint32_t off = tme_trim - tme_min;
	for (uint32_t n = 0; n < tme_min; ++n) {
		float g = 1.f - (n / (float)tme_min);
		for (uint32_t c = 0; c < n_channels; ++c) {
			data[c][off + n] *= g;
		}
	}

	for (uint32_t c = 0; c < n_channels; ++c) {
		memset (&data[c][tme_trim], 0, sizeof (float) * (n_samples - tme_trim));
	}

	return tme_trim;
}

static float
digital_peak (uint32_t n_channels, uint32_t n_samples, float** data)
{
	float sig_max = 0;
	for (uint32_t c = 0; c < n_channels; ++c) {
		for (uint32_t n = 0; n < n_samples; ++n) {
			float s = fabsf (data[c][n]);
			if (s > sig_max) {
				sig_max = s;
			}
		}
	}
	return sig_max;
}

static float
normalize_peak (uint32_t n_channels, uint32_t n_samples, float** data)
{
	float sig_max = digital_peak (n_channels, n_samples, data);
	float target  = exp10f (.05 * -3);

	if (sig_max == 0 || sig_max > target) {
		return 1.0;
	}

	const float g = target / sig_max;
	for (uint32_t c = 0; c < n_channels; ++c) {
		for (uint32_t n = 0; n < n_samples; ++n) {
			data[c][n] *= g;
		}
	}
	return g;
}

static uint32_t
gensweep (float fmin, float fmax, float t_sec, float rate)
{
	int n_samples_pre = rate * 0.1f;
	int n_samples_sin = rate * t_sec;
	int n_samples_end = rate * 0.03f;

	int n_samples = n_samples_pre + n_samples_sin + n_samples_end;

	free (sweep_sin);
	free (sweep_inv);
	sweep_sin = (float*)malloc (sizeof (float) * n_samples);
	sweep_inv = (float*)malloc (sizeof (float) * n_samples);

	double amp = 0.5;

	double a = log (fmax / fmin) / (double)n_samples_sin;
	double b = fmin / (a * rate);
	double r = 4.0 * a * a / amp;

	for (int i = 0; i < n_samples; ++i) {
		int j = n_samples - i - 1;

		double gain = 1.0;
		if (i < n_samples_pre) {
			gain = sin (0.5 * M_PI * i / n_samples_pre);
		} else if (j < n_samples_end) {
			gain = sin (0.5 * M_PI * j / n_samples_end);
		}

		double d = b * exp (a * (i - n_samples_pre));
		double p = d - b;
		double x = gain * sin (2.f * M_PI * (p - floor (p)));

		sweep_sin[i] = x * amp;
		sweep_inv[j] = x * d * r;
	}
	return n_samples;
}

static void
cleanup ()
{
	free (input_ports);
	free (output_ports);
	free (sweep_sin);
	free (sweep_inv);

	for (uint32_t n = 0; ir && n < n_ir; ++n) {
		free (ir[n]);
	}
	free (ir);
}

static void
catchsig (int sig)
{
	fprintf (stderr, "caught signal - shutting down.\n");
	client_state = Abort;
}

static bool
file_exists (std::string const& name)
{
	struct stat buffer;
	return (stat (name.c_str (), &buffer) == 0);
}

static void
print_usage (void)
{
	printf ("jack-ir - JACK Impulse Response Capture Utility\n\n");
	printf ("Usage: jack_ir [ OPTIONS ] [ OUT-FILE ]\n\n");
	printf (""
	        "This is a standalone JACK application to conveniently capture impulse\n"
	        "responses of external devices.\n"
	        "\n"
	        "The tool supports four different IR file configurations.\n"
	        " * Mono:            1 in, 1 out\n"
	        " * Mono-to-Stereo:  1 in, 2 out\n"
	        " * Stereo:          2 in, 2 out\n"
	        " * True-Stereo:     2 in, 2 out, 4channels (L->L, L->R, R->L, R->R)\n"
	        "\n"
	        "The configuration happens indirectly by specifying the capture and playback"
	        "ports to be used when recording the IR.\n"
	        "The impulse-response is captured by playing a sine-sweep chirp via the\n"
	        "configured playback port(s) while recording the response of the system under\n"
	        "test from the configured capture-port(s).\n"
	        "\n"
	        "The default is to record the response from a 10 sec chirp for 15sec.\n"
	        "For true-stereo this process is repeated to capture responses for left,and\n"
	        "right channels separately.\n"
	        "\n"
	        "Eventually the IR is computed, normalized and trimmed and saved as wav file.\n"
	        "\n"
	        "Note that this tool is meant for patch-processing of directly connected\n"
	        "hardware effect units. In order to properly align the IR, it should be used\n"
	        "with latency-calibrated jackd.\n"
	        "\n"
	        "For capturing rooms, or setups with involving microphones and speakers\n"
	        "do prefer manual capture, equalization and post-processing e.g. using aliki.\n");

	/* **** "---------|---------|---------|---------|---------|---------|---------|---------|" */
	printf ("\n"
	        "Options:\n"
	        " -h, --help                Display this help and exit\n"
	        " -c, --capture <port>      Add channel, specify source-port to connect to\n"
	        " -C <sec>                  Max capture length (default 15s)\n"
	        " -p, --playback <port>     Add playback-port to connect to\n"
	        " -j, --jack-name <name>    Set the JACK client name\n"
	        " -L, --latency <int>       Specify custom round-trip latency (audio-samples)\n"
	        " -S <sec>                  Silence between true-stereo captures (default: 1s)\n"
	        " -T, --true-stereo         4 channel, true stereo IR. This needs 2 capture,\n"
	        "                           and 2 playback channels.\n"
	        " -q, --quiet               Inhibit non-error messages\n"
	        " -V, --version             Print version information and exit\n"
	        " -y, --overwrite           Replace output file if it exists\n"
	        "If the OUT-FILE parameter is not given, 'ir.wav' is used.\n");

	printf ("\n"
	        "Examples:\n"
	        "jack-ir -c system:capture_1 -p system:playback_1\n\n"
	        "jack-ir -c system:capture_1 -c system:capture_2 -p system:playback_1 mono_to_stereo.wav\n\n"
	        "jack-ir -T -c system:capture_3 -c system:capture_4 -p system:playback_5 -p system:playback_6\n\n");

	printf ("Report bugs at <https://github.com/x42/jack-ir/issues>\n");
	printf ("Website: <http://github.com/x42/jack-ir>\n");
}

static void
print_version (void)
{
	printf ("jack-ir version %s\n\n", VERSION);
	printf ("\n"
	        "Copyright (C) GPL 2019 Robin Gareus <robin@gareus.org>\n"
	        "This is free software; see the source for copying conditions.  There is NO\n"
	        "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n\n");
}

int
main (int argc, char** argv)
{
	int            rv          = -1;
	uint32_t       n_max       = 0;
	const char*    client_name = "ir";
	int            latency     = 0;
	bool           overwrite   = false;
	bool           quiet       = false;
	bool           xrun_abort  = true;
	jack_options_t options     = JackNoStartServer;
	jack_status_t  status;

	float sweep_min = 20.f;    // Hz
	float sweep_max = 20000.f; // Hz
	float sweep_sec = 10.f;    // sec (without fades)
	float irrec_sec = 15.f;    // sec
	float t_silence = 1.f;     // sec

	std::string outfile = "ir.wav";

	std::vector<std::string> capt;
	std::vector<std::string> play;

	/* clang-format off */
	const struct option long_options[] = {
		{ "capture",   required_argument, 0, 'c' },
		{ "help",      no_argument,       0, 'h' },
		{ "jack-name", required_argument, 0, 'j' },
		{ "latency",   required_argument, 0, 'L' },
		{ "playback",  required_argument, 0, 'p' },
		{ "quiet",     no_argument,       0, 'q' },
		{ "version",   no_argument,       0, 'V' },
		{ "overwrite", no_argument,       0, 'y' },
	};
	/* clang-format on */

	const char* optstring = "C:c:hj:L:p:S:TqVy";

	int c;
	while ((c = getopt_long (argc, argv, optstring, long_options, NULL)) != -1) {
		switch (c) {
			case 'C':
				irrec_sec = atof (optarg);
				break;
			case 'c':
				capt.push_back (optarg);
				break;
			case 'h':
				print_usage ();
				return 0;
				break;
			case 'j':
				client_name = optarg;
				break;
			case 'L':
				latency = atoi (optarg);
				break;
			case 'p':
				play.push_back (optarg);
				break;
			case 'S':
				t_silence = std::min (10.f, std::max (1.f, (float)atof (optarg)));
				break;
			case 'T':
				true_stereo = true;
				break;
			case 'q':
				quiet = true;
				break;
			case 'V':
				print_version ();
				return 0;
				break;
			case 'y':
				overwrite = true;
				break;
			default:
				fprintf (stderr, "Invalid argument.\n");
				print_usage ();
				return (1);
				break;
		}
	}

	if (optind > argc) {
		fprintf (stderr, "Invalid argument.\n");
		print_usage ();
		return 1;
	}

	if (optind < argc) {
		outfile = argv[optind];
	}

	n_inputs  = capt.size ();
	n_outputs = play.size ();

	if (n_outputs < 1 || n_outputs > 2 || n_inputs < 1 || n_inputs > 2 || n_outputs > n_inputs) {
		fprintf (stderr, "Invalid number of i/o ports\n");
		return -1;
	}

	if (n_outputs != 2 || n_inputs != 2) {
		if (true_stereo) {
			fprintf (stderr, "True-Stereo needs stereo I/O\n");
			return -1;
		}
	}

	if (irrec_sec < sweep_sec + .5f || irrec_sec > 30.f) {
		fprintf (stderr, "Capture lenght is out of bounds %.1f < len <= 30.0 [sec]\n", sweep_sec + .5f);
		return -1;
	}

	if (file_exists (outfile)) {
		if (!overwrite) {
			fprintf (stderr, "Error: IR file exists ('%s')\n", outfile.c_str ());
			return -1;
		}
		fprintf (stderr, "Warning: replacing IR ('%s')\n", outfile.c_str ());
	}

	if (true_stereo) {
		assert (n_outputs == 2 && n_inputs == 2);
		n_ir = 4;
	} else {
		n_ir = n_inputs;
	}

	/* open a client connection to the JACK server */
	jack_client_t* j_client = jack_client_open (client_name, options, &status, NULL);

	if (!j_client) {
		fprintf (stderr, "jack_client_open() failed (status 0x%x)\n", status);
		if (status & JackServerFailed) {
			fprintf (stderr, "Unable to connect to JACK server\n");
		}
		return -1;
	}

	jack_set_process_callback (j_client, jack_process, 0);
	jack_set_graph_order_callback (j_client, jack_graph_order_cb, 0);
	jack_on_shutdown (j_client, jack_shutdown, 0);
	if (xrun_abort) {
		jack_set_xrun_callback (j_client, jack_xrun, 0);
	}

	/* display the current sample rate. */
	uint32_t rate = jack_get_sample_rate (j_client);
	if (!quiet) {
		printf ("Engine sample rate: %" PRIu32 "\n", rate);
	}
	if (rate < 44100 || rate > 96000) {
		fprintf (stderr, "Invalid sample-rate, not (44100 <= rate <= 96000)\n");
		goto out;
	}

	if (true_stereo) {
		true_stereo_pass = rate * t_silence;
	}

	/* prepare sweep */
	irrec_len = irrec_sec * rate;
	sweep_len = gensweep (sweep_min, sweep_max, sweep_sec, rate);

#if 0 // Debug Dump sweep
	{
		float* sd[2] = { sweep_sin, sweep_inv };
		sf_write ("/tmp/ir_sweep.wav", 2, rate, 0, sweep_len, sd);
	}
#endif

	ir           = (float**)calloc (n_ir, sizeof (float*));
	input_ports  = (jack_port_t**)calloc (n_inputs, sizeof (jack_port_t*));
	output_ports = (jack_port_t**)calloc (n_outputs, sizeof (jack_port_t*));

	if (!ir || !input_ports || !output_ports) {
		fprintf (stderr, "Out of Memory\n");
		goto out;
	}

	for (uint32_t n = 0; n < n_outputs; ++n) {
		char tmp[64];
		snprintf (tmp, sizeof (tmp), "sweep_%d", n + 1);
		output_ports[n] = jack_port_register (j_client, tmp,
		                                      JACK_DEFAULT_AUDIO_TYPE,
		                                      JackPortIsOutput, 0);

		if (!output_ports[n]) {
			fprintf (stderr, "No more JACK ports available\n");
			goto out;
		}
	}

	for (uint32_t n = 0; n < n_inputs; ++n) {
		char tmp[64];
		snprintf (tmp, sizeof (tmp), "input_%d", n + 1);
		input_ports[n] = jack_port_register (j_client, tmp,
		                                     JACK_DEFAULT_AUDIO_TYPE,
		                                     JackPortIsInput, 0);

		if (!input_ports[n]) {
			fprintf (stderr, "No more JACK ports available\n");
			goto out;
		}
	}

	for (uint32_t n = 0; n < n_ir; ++n) {
		ir[n] = (float*)calloc (sweep_len + irrec_len, sizeof (float));
		if (!ir[n]) {
			fprintf (stderr, "Out of Memory\n");
			goto out;
		}
	}

#if 0 // DEBUG test convolv
	for (uint32_t n = 0; n < n_ir; ++n) {
		memcpy (ir[n], sweep_sin, sweep_len * sizeof (float));
	}
	if (convolv (n_ir, sweep_len + irrec_len, ir)) { goto out; }
	rv = sf_write ("/tmp/ir_conv.wav", n_ir, rate, 0, sweep_len + irrec_len, ir);
	goto out;
#endif

	if (jack_activate (j_client)) {
		fprintf (stderr, "Cannot activate JACK client");
		goto out;
	}

	/* connect ports */
	for (uint32_t n = 0; n < n_outputs; ++n) {
		jack_connect (j_client, jack_port_name (output_ports[n]), play[n].c_str ());
	}

	for (uint32_t n = 0; n < n_inputs; ++n) {
		jack_connect (j_client, capt[n].c_str (), jack_port_name (input_ports[n]));
	}

	n_max = irrec_len;
	if (true_stereo) {
		n_max += irrec_len + true_stereo_pass;
	}

#ifndef _WIN32
	signal (SIGHUP, catchsig);
	signal (SIGINT, catchsig);
#endif

	sleep (1);
	client_state = Run;

	if (!quiet) {
		if (latency > 0) {
			printf ("JACK round-trip latency: %d (ignored, using %d)\n", roundtrip_latency, latency);
		} else {
			printf ("Round-trip latency: %d\n", roundtrip_latency);
		}
	}

	while (client_state == Run) {
		sleep (1);
		if (!quiet) {
			printf ("Processing: %3.0f%% (%c) \r",
			        std::min (100.f, 100.f * proc_tot / n_max),
			        proc_pos < sweep_len ? 'P' : 'C');
			fflush (stdout);
		}
	}
	if (!quiet) {
		printf ("\n");
	}

	/* post-process, if capture was not aborted */
	if (client_state == Exit) {
		float in_peak = digital_peak (n_ir, sweep_len + irrec_len, ir);

		if (!quiet) {
			printf ("Input signal peak: %.2fdBFS\n", 20 * log (in_peak));
		}

		if (in_peak >= .98) {
			fprintf (stderr, "Input signal clipped!\n");
			goto out;
		}

		if (convolv (n_ir, sweep_len + irrec_len, ir)) {
			fprintf (stderr, "Deconvolution failed\n");
			goto out;
		}

		float g = normalize_peak (n_ir, sweep_len + irrec_len, ir);
		if (!quiet) {
			printf ("Normalized IR, gain-factor: %.2fdB\n", 20 * log (g));
		}

		uint32_t trimed_len = trim_end (n_ir, rate, sweep_len + irrec_len, ir);

		int lat = 0;
		if (latency > 0) {
			lat = latency;
		}
#if 1 /* allow for some io-delay inaccuracy and sinc pre-ringing */
		else if (roundtrip_latency > 3) {
			lat = roundtrip_latency - 4;
		}
#endif
		else {
			lat = roundtrip_latency;
		}

		if (trimed_len < sweep_len + lat) {
			fprintf (stderr, "IR is too short or empty\n");
		} else {
			uint32_t ir_len = trimed_len - (sweep_len + lat);
			if (!quiet) {
				printf ("Writing IR: %d channels, %.1f [sec] = %d [spl] '%s'\n", n_ir, ir_len / (float)rate, ir_len, outfile.c_str ());
			}
			rv = sf_write (outfile.c_str (), n_ir, rate, sweep_len + lat, ir_len, ir);
		}
	}

out:
	jack_client_close (j_client);
	cleanup ();
	return rv;
}
