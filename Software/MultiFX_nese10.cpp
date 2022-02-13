#include "daisy_seed.h"
#include "daisysp.h"

// Set max delay time to 0.75 of samplerate.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)

using namespace daisy;
using namespace daisysp;

static DaisySeed hw;

static Phaser										phas;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS 	dell;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS 	delr;
static Tremolo										trem;
static Parameter deltime;

float currentDelay, feedback, delayTarget, cutoff, dryWet[4];
bool effectOn[3];

enum FX_types {
	PHAS,
	DEL,
	TREM,
	// DRY,
	// ALL,
};

FX_types which_FX;	//Here is stored the value of the actual effect

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		out[0][i] = in[0][i];
		out[1][i] = in[1][i];
	}
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
	while(1) {}
}
