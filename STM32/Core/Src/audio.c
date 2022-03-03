/*
 * audio_processing.c
 *
 *  Created on: May 17, 2021
 *      Author: sydxrey
 *
 *
 * === Audio latency ===
 *
 * Receive DMA copies audio samples from the CODEC to the DMA buffer (through the I2S serial bus) as interleaved stereo samples
 * (either slots 0 and 2 for the violet on-board "LineIn" connector, or slots 1 and 3, for the pair of on-boards microphones).
 *
 * Transmit DMA copies audio samples from the DMA buffer to the CODEC again as interleaved stereo samples (in the present
 * implementation only copying to the headphone output, that is, to slots 0 and 2, is available).
 *
 * For both input and output transfers, audio double-buffering is simply implemented by using
 * a large (receive or transmit) buffer of size AUDIO_DMA_BUF_SIZE
 * and implementing half-buffer and full-buffer DMA callbacks:
 * - HAL_SAI_RxHalfCpltCallback() gets called whenever the receive (=input) buffer is half-full
 * - HAL_SAI_RxCpltCallback() gets called whenever the receive (=input) buffer is full
 *
 * As a result, one audio frame has a size of AUDIO_DMA_BUF_SIZE/2. But since one audio frame
 * contains interleaved L and R stereo samples, its true duration is AUDIO_DMA_BUF_SIZE/4.
 *
 * Example:
 * 		AUDIO_BUF_SIZE = 512 (=size of a stereo audio frame)
 * 		AUDIO_DMA_BUF_SIZE = 1024 (=size of the whole DMA buffer)
 * 		The duration of ONE audio frame is given by AUDIO_BUF_SIZE/2 = 256 samples, that is, 5.3ms at 48kHz.
 *
 * === interprocess communication ===
 *
 *  Communication b/w DMA IRQ Handlers and the main audio loop is carried out
 *  using the "audio_rec_buffer_state" global variable (using the input buffer instead of the output
 *  buffer is a matter of pure convenience, as both are filled at the same pace anyway).
 *
 *  This variable can take on three possible values:
 *  - BUFFER_OFFSET_NONE: initial buffer state at start-up, or buffer has just been transferred to/from DMA
 *  - BUFFER_OFFSET_HALF: first-half of the DMA buffer has just been filled
 *  - BUFFER_OFFSET_FULL: second-half of the DMA buffer has just been filled
 *
 *  The variable is written by HAL_SAI_RxHalfCpltCallback() and HAL_SAI_RxCpltCallback() audio in DMA transfer callbacks.
 *  It is read inside the main audio loop (see audioLoop()).
 *
 *  If RTOS is to used, Signals may be used to communicate between the DMA IRQ Handler and the main audio loop audioloop().
 *
 */

#include <audio.h>
#include <ui.h>
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "bsp/disco_sai.h"
#include "bsp/disco_base.h"

extern SAI_HandleTypeDef hsai_BlockA2; // see main.c
extern SAI_HandleTypeDef hsai_BlockB2;
extern DMA_HandleTypeDef hdma_sai2_a;
extern DMA_HandleTypeDef hdma_sai2_b;


// ---------- communication b/w DMA IRQ Handlers and the audio loop -------------

typedef enum {
	BUFFER_OFFSET_NONE = 0, BUFFER_OFFSET_HALF = 1, BUFFER_OFFSET_FULL = 2,
} BUFFER_StateTypeDef;
uint32_t audio_rec_buffer_state;




// ---------- DMA buffers ------------

// whole sample count in an audio frame: (beware: as they are interleaved stereo samples, true audio frame duration is given by AUDIO_BUF_SIZE/2)
#define AUDIO_BUF_SIZE   ((uint32_t)512)
/* size of a full DMA buffer made up of two half-buffers (aka double-buffering) */
#define AUDIO_DMA_BUF_SIZE   (2 * AUDIO_BUF_SIZE)

// DMA buffers are in embedded RAM:
int16_t buf_input[AUDIO_DMA_BUF_SIZE];
int16_t buf_output[AUDIO_DMA_BUF_SIZE];
int16_t *buf_input_half = buf_input + AUDIO_DMA_BUF_SIZE / 2;
int16_t *buf_output_half = buf_output + AUDIO_DMA_BUF_SIZE / 2;


// DELAY BUFFERS

#define AUDIO_SDRAM_SIZE ((uint32_t)1024000)


// ------------- scratch float buffer for long delays, reverbs or long impulse response FIR based on float implementations ---------

uint32_t scratch_offset = 0; // see doc in processAudio()
#define AUDIO_SCRATCH_SIZE   AUDIO_SCRATCH_MAXSZ_WORDS



// ------------ Private Function Prototypes ------------

static void processAudio(int16_t* out, int16_t* in);
static void processAudioSDRAM(int16_t* out);
static void processAudioSDRAM_Delay(int16_t* out);
static void storeInSDRAM(int16_t *in);

static void accumulateInputLevels();
static float readFloatFromSDRAM(int pos);
static void writeFloatToSDRAM(float val, int pos);

// ----------- Local vars ------------

static int count = 0; // debug
static double inputLevelL = 0;
static double inputLevelR = 0;

// ----------- Functions ------------

/**
 * This is the main audio loop (aka infinite while loop) which is responsible for real time audio processing tasks:
 * - transferring recorded audio from the DMA buffer to buf_input[]
 * - processing audio samples and writing them to buf_output[]
 * - transferring processed samples back to the DMA buffer
 */
void audioLoop() {

	uiDisplayBasic();

	/* Initialize SDRAM buffers */
	memset((int16_t*) AUDIO_SCRATCH_ADDR, 0, AUDIO_SCRATCH_SIZE * 2); // note that the size argument here always refers to bytes whatever the data type

	audio_rec_buffer_state = BUFFER_OFFSET_NONE;

	// start SAI (audio) DMA transfers:
	startAudioDMA(buf_output, buf_input, AUDIO_DMA_BUF_SIZE);

	/* main audio loop */
	while (1) {
		/* calculate average input level over 20 audio frames */
		accumulateInputLevels();
		count++;
		if (count >= 20) {
			count = 0;
			inputLevelL *= 0.05;
			inputLevelR *= 0.05;
			uiDisplayInputLevel(inputLevelL, inputLevelR);
			inputLevelL = 0.;
			inputLevelR = 0.;
		}

		/* Wait until first half block has been recorded */
		while (audio_rec_buffer_state != BUFFER_OFFSET_HALF) {
			asm("NOP");
		}
		audio_rec_buffer_state = BUFFER_OFFSET_NONE;
		/* Copy recorded 1st half block */
		storeInSDRAM(buf_input);
		//processAudioSDRAM(buf_output, buf_input);
		processAudioSDRAM_Delay(buf_output);

		/* Wait until second half block has been recorded */
		while (audio_rec_buffer_state != BUFFER_OFFSET_FULL) {
			asm("NOP");
		}
		audio_rec_buffer_state = BUFFER_OFFSET_NONE;
		/* Copy recorded 2nd half block */
		storeInSDRAM(buf_input_half);
		//processAudioSDRAM(buf_output_half, buf_input_half);
		processAudioSDRAM_Delay(buf_output_half);
	}
}


/*
 * Update input levels from the last audio frame (see global variable inputLevelL and inputLevelR).
 * Reminder: audio samples are actually interleaved L/R samples,
 * with left channel samples at even positions,
 * and right channel samples at odd positions.
 */
static void accumulateInputLevels() {

	// Left channel:
	uint32_t lvl = 0;
	for (int i = 0; i < AUDIO_DMA_BUF_SIZE; i += 2) {
		int16_t v = (int16_t) buf_input[i];
		if (v > 0)
			lvl += v;
		else
			lvl -= v;
	}
	inputLevelL += (double) lvl / AUDIO_DMA_BUF_SIZE / (1 << 15);

	// Right channel:
	lvl = 0;
	for (int i = 1; i < AUDIO_DMA_BUF_SIZE; i += 2) {
		int16_t v = (int16_t) buf_input[i];
		if (v > 0)
			lvl += v;
		else
			lvl -= v;
	}
	inputLevelR += (double) lvl / AUDIO_DMA_BUF_SIZE / (1 << 15);
	;
}

// --------------------------- Callbacks implementation ---------------------------

/**
 * Audio IN DMA Transfer complete interrupt.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	audio_rec_buffer_state = BUFFER_OFFSET_FULL;
	return;
}

/**
 * Audio IN DMA Half Transfer complete interrupt.
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	audio_rec_buffer_state = BUFFER_OFFSET_HALF;
	return;
}

/* --------------------------- Audio "scratch" buffer in SDRAM ---------------------------
 *
 * The following functions allows you to use the external SDRAM as a "scratch" buffer.
 * There are around 7Mbytes of RAM available (~ 1' of stereo sound) which makes it possible to store signals
 * (either input or processed) over long periods of time for e.g. FIR filtering or long tail reverb's.
 */

/**
 * Read a 32 bit float from SDRAM at position "pos"
 */
static float readFloatFromSDRAM(int pos) {

	__IO float *pSdramAddress = (float*) AUDIO_SCRATCH_ADDR; // __IO is used to specify access to peripheral variables
	pSdramAddress += pos;
	//return *(__IO float*) pSdramAddress;
	return *pSdramAddress;

}

/**
 * Write the given 32 bit float to SDRAM at position "pos"
 */
static void writeFloatToSDRAM(float val, int pos) {

	__IO float *pSdramAddress = (float*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//*(__IO float*) pSdramAddress = val;
	*pSdramAddress = val;

}

/**
 * Read a 16 bit integer from SDRAM at position "pos"
 */
static int16_t readInt16FromSDRAM(int pos) {

	__IO int16_t *pSdramAddress = (int16_t*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//return *(__IO int16_t*) pSdramAddress;
	return *pSdramAddress;

}

/**
 * Write the given 16 bit integer to the SDRAM at position "pos"
 */
static void writeInt16ToSDRAM(int16_t val, int pos) {

	__IO int16_t *pSdramAddress = (int16_t*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//*(__IO int16_t*) pSdramAddress = val;
	*pSdramAddress = val;

}

// --------------------------- AUDIO ALGORITHMS ---------------------------

/**
 * This function is called every time an audio frame
 * has been filled by the DMA, that is,  AUDIO_BUF_SIZE samples
 * have just been transferred from the CODEC
 * (keep in mind that this number represents interleaved L and R samples,
 * hence the true corresponding duration of this audio frame is AUDIO_BUF_SIZE/2 divided by the sampling frequency).
 * 5.3333 ms
 */
static void processAudio(int16_t *out, int16_t *in) {

	LED_On(); // for oscilloscope measurements...

	for (int n = 0; n < AUDIO_BUF_SIZE; n++) out[n] = in[n];

	LED_Off();
}

int i_writeSDRAM = 0;
int i_readSDRAM = 0;
int i_readDelaySDRAM = 0;

int i_readWriteSDRAM_Out = AUDIO_SDRAM_SIZE;
int i_readDelaySDRAM_Out = AUDIO_SDRAM_SIZE;

static void processAudioSDRAM(int16_t *out){
	LED_On(); // for oscilloscope measurements...

	for (int n = 0; n < AUDIO_BUF_SIZE; n++){
		//writeInt16ToSDRAM(in[n], i_WriteReadSDRAM);
		out[n] = readInt16FromSDRAM(i_readSDRAM);
		i_readSDRAM++;

		if(i_readSDRAM >= AUDIO_SDRAM_SIZE){
			i_readSDRAM = 0;
		}
	}

	LED_Off();
}


/*
 * 1 échantillon = 10.42 us
 * 1 delay de 0.5s -> 42985
 * Si N impair -> mélange audio gauche et droite
 */
static void processAudioSDRAM_Delay(int16_t *out) {
	int N = 47984;
	float fb = 0.60;
	float D = 0.75;
	float W = 0.25;

	for (int n = 0; n < AUDIO_BUF_SIZE; n++){
		if((i_readSDRAM-N) < 0){
			i_readDelaySDRAM = ((i_readSDRAM-N)*(-1))%AUDIO_SDRAM_SIZE;
		}else{
			i_readDelaySDRAM = (i_readSDRAM-N);
		}

		if((i_readWriteSDRAM_Out - N) < AUDIO_SDRAM_SIZE){
			i_readDelaySDRAM_Out = i_readWriteSDRAM_Out + AUDIO_SDRAM_SIZE;
		}else{
			i_readDelaySDRAM_Out = (i_readWriteSDRAM_Out-N);
		}

		out[n] = readInt16FromSDRAM(i_readDelaySDRAM_Out) * fb;
		out[n] += readInt16FromSDRAM(i_readSDRAM);
		out[n] -= readInt16FromSDRAM(i_readDelaySDRAM) * D * fb;
		out[n] += readInt16FromSDRAM(i_readDelaySDRAM) * W;

		writeInt16ToSDRAM(out[n], i_readWriteSDRAM_Out);

		i_readSDRAM++;
		i_readWriteSDRAM_Out++;
		if(i_readSDRAM >= AUDIO_SDRAM_SIZE){
			i_readSDRAM = 0;
		}
		if(i_readWriteSDRAM_Out >= (AUDIO_SDRAM_SIZE + AUDIO_SDRAM_SIZE)){
			i_readWriteSDRAM_Out = AUDIO_SDRAM_SIZE;
		}

	}
}

static void storeInSDRAM(int16_t *in){
	for (int n = 0; n < AUDIO_BUF_SIZE; n++){
		writeInt16ToSDRAM(in[n], i_writeSDRAM);
		i_writeSDRAM++;
		if(i_writeSDRAM >= AUDIO_SDRAM_SIZE){
			i_writeSDRAM = 0;
		}
	}
}


