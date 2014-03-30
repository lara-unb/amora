#ifndef CONTINUOUS_H
#define CONTINUOUS_H
#include <string>
#include <iostream>
#include <stdexcept>

#include <sys/types.h>
#include <sys/time.h>

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/cont_ad.h>

#include "pocketsphinx.h"

using namespace std;

//Class decoder for speech recognizer
class decoder
{
	private:
		//Configuration
		cmd_ln_t *config;

		//Decoder
		ps_decoder_t *ps;
		
		//A/D Converter
		ad_rec_t *ad;
		
		char const *hyp;
		
		//utterrance id
		char const *uttid;

		cont_ad_t *cont;
		
		//A/D Buffer
		int16 adbuf[4096];
		
		int32 k, ts, rem;

		//Sleep for specified msec
		void sleep_msec(int32 ms);
		
		//Initialize continuous listening module
		void init()	throw(runtime_error);
		
		//Release memory
		void release();
	public:
		decoder()	throw(runtime_error);
		
		string recognize()	throw(runtime_error);
		
		~decoder();
};

#endif //CONTINUOUS_H
