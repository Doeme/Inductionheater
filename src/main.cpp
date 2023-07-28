#include <driver/mcpwm.h>
#include <NeoPixelBus.h>
NeoPixelBus<NeoRgbFeature, NeoEsp32Rmt1800KbpsMethod> strip(1,  38); //, 17, 16);


void setup(){
	strip.Begin();
 	strip.ClearTo(RgbColor(255,255,255));
	strip.Show();

	ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4));
	ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 5));

	mcpwm_config_t cfg;
	cfg.frequency = 25000;
	cfg.cmpr_a = 50;
	cfg.cmpr_b = 50;
	cfg.duty_mode = MCPWM_DUTY_MODE_0;
	cfg.counter_mode = MCPWM_UP_COUNTER;

	ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg));
	//mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50));
	//mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);

}

// 10us on time
// 3.5us dead time

void doPulse() {

}

void loop(){
	static int cnt = 0;
	if(cnt++ >= 1000){
		cnt = 0;
	}
	float f = ((float)cnt)/1000.0;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, f*100);
	/*delayMicroseconds(1000000);
	mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
	delayMicroseconds(1000000);
	mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);*/
	delayMicroseconds(1000);
	strip.ClearTo(RgbColor(0,(int)(f*255.0),0));
    strip.Show();
}
