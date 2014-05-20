#include "continuous.h"

/* Parameters */
const arg_t cont_args_def[] = {
				POCKETSPHINX_OPTIONS,
				{ "-adcdev", 
				  ARG_STRING, 
				  NULL, 
				  "Name of audio device to use for input." },
				CMDLN_EMPTY_OPTION
			      };
char *argv[] = {(char*)"./continuous",
		(char*)"-adcdev",
		(char*)"plughw:0,0",
		(char*)"-lm",
		(char*)"/home/gastd/workspaces/hydro/catkin_ws/src/fsm/data/model/language_model.lm",
		(char*)"-dict",
		(char*)"/home/gastd/workspaces/hydro/catkin_ws/src/fsm/data/model/phonetic_dictionary.dic",
		};
int argc = 7;


decoder::decoder()	throw(std::runtime_error)
{
	init();
}

decoder::~decoder()
{
	release();
}


void decoder::sleep_msec(int32 ms)
{
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
}


std::string decoder::recognize()	throw(std::runtime_error)
{
/*
 * Main utterance processing loop:
 *     for (;;) {
 * 	   wait for start of next utterance;
 * 	   decode utterance until silence of at least 1 sec observed;
 * 	   print utterance result;
 *     }
 */
 
	//Indicate listening for next utterance
	std::cout << "READY...." << std::endl;

	fflush(stdout);
	fflush(stderr);
	
	//Wait data for next utterance
	while ((k = cont_ad_read(cont, adbuf, 4096)) == 0)	sleep_msec(100);
	
	/*
         * Non-zero amount of data received; start recognition of new utterance.
         * NULL argument to uttproc_begin_utt => automatic generation of utterance-id.
         */
	if (ps_start_utt(ps, NULL) < 0)		throw std::runtime_error("Failed to start utterance\n");
	ps_process_raw(ps, adbuf, k, FALSE, FALSE);

	std::cout << "Listening..." << std::endl;
	fflush(stdout);

	//Note timestamp for this first block of data
	ts = cont->read_ts;

	//Decode utterance until end (marked by a "long" silence, >1sec)
	for(;;)
	{
		//Read non-silence audio data, if any, from continuous listening module
		if ((k = cont_ad_read(cont, adbuf, 4096)) < 0)	throw std::runtime_error("Failed to read audio\n");
	
		if (k == 0)
		{
			/*
		        /* No speech data available; check current timestamp with most recent
		         * speech to see if more than 1 sec elapsed.  If so, end of utterance.
		         */
			if ((cont->read_ts - ts) > DEFAULT_SAMPLES_PER_SEC)	break;
		}
		else
		{
			//New speech data received; note current timestamp
			ts = cont->read_ts;
		}
	
		//Decode whatever data was read above.
		rem = ps_process_raw(ps, adbuf, k, FALSE, FALSE);
	
		if ((rem == 0) && (k == 0))	sleep_msec(20);
	}
	
	/*
         * Utterance ended; flush any accumulated, unprocessed A/D data and stop
         * listening until current utterance completely decoded
         */
	ad_stop_rec(ad);
	while (ad_read(ad, adbuf, 4096) >= 0);
	cont_ad_reset(cont);

	std::cout << "Stopped listening, please wait..." << std::endl;
	fflush(stdout);

	//Finish decoding, obtain and print result
	ps_end_utt(ps);
	hyp = ps_get_hyp(ps, NULL, &uttid);
	std::string str(hyp);
	fflush(stdout);
	
	//Resume A/D recording for next utterance
	if (ad_start_rec(ad) < 0)	throw std::runtime_error("Failed to start recording\n");
	return str;
}

void decoder::init()	throw(std::runtime_error)
{
	config = cmd_ln_parse_r(NULL, cont_args_def, argc, argv, FALSE);
		
	if(config == NULL)	throw  std::runtime_error("Failed to configure device\n");
		
	ps = ps_init(config);
	
	if(ps == NULL)	throw std::runtime_error("Failed to initialize device\n");
		
	if ( (ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"), (int)cmd_ln_float32_r(config, "-samprate"))) == NULL )	throw std::runtime_error("Failed to open audio device\n");

	if ((cont = cont_ad_init(ad, ad_read)) == NULL)	throw std::runtime_error("Failed to initialize voice activity detection\n");
	
	if (ad_start_rec(ad) < 0)	throw std::runtime_error("Failed to start recording\n");
	
	if (cont_ad_calib(cont) < 0)	throw std::runtime_error("Failed to calibrate voice activity detection\n");
}

void decoder::release()
{
	cont_ad_close(cont);
	ad_close(ad);
	ps_free(ps);
	//cmd_ln_free_r(config);
	cmd_ln_free();
	E_FATAL("oi");
}
