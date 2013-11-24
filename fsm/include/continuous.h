//Header Continuous
#ifndef CONTINUOUS_H
#define CONTINUOUS_H
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

#if !defined(_WIN32_WCE)
#include <signal.h>
#include <setjmp.h>
#endif
#if defined(WIN32) && !defined(GNUWINCE)
#include <time.h>
#else
#include <sys/types.h>
#include <sys/time.h>
#endif

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/cont_ad.h>

#include "pocketsphinx.h"
#include "festival.h"
#include "filters.h"

using namespace std;

namespace ps
{
	const arg_t cont_args_def[] =
	{
	    POCKETSPHINX_OPTIONS,
	    /* Argument file. */
	    { "-argfile",
	      ARG_STRING,
	      NULL,
	      "Argument file giving extra arguments." },
	    { "-adcdev", 
	      ARG_STRING, 
	      "plughw:1,0\0",
	      "Name of audio device to use for input." },
	    { "-infile", 
	      ARG_STRING, 
	      NULL, 
	      "Audio file to transcribe." },
	    { "-time", 
	      ARG_BOOLEAN, 
	      "no", 
	      "Print word times in file transcription." },
	    CMDLN_EMPTY_OPTION
	};

	ps_decoder_t *ps;
	cmd_ln_t *config;
	jmp_buf jbuf;

	void sleep_msec(int32);

	void recognize_from_microphone(string&);

	int continuous(int , char **, string&);
}

#endif
