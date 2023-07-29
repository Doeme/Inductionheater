#include <driver/mcpwm.h>
#include <NeoPixelBus.h>
NeoPixelBus<NeoRgbFeature, NeoEsp32Rmt1800KbpsMethod> strip(1,  38); //, 17, 16);



enum ButtonResult{
	None,
	ButtonPressed,
	ButtonPressing,
	ButtonReleased
};

class Button{
	bool state;
	unsigned long blank = 0;
	int blanking = 10;
	int waiting = 400;
	int step = 50;
	int steps = 0;
	
	public:
	ButtonResult feed(bool new_state){
		unsigned long now = millis();
		if((state != new_state) && now > blank ){
			blank = now + blanking;
			state = new_state;
			if(state){
				steps = 0;
				return ButtonPressed;
			}
			else{
				return ButtonReleased;
			}
		}
		
		if(new_state && (now > blank+(waiting-blanking)) ){
			long dt = now-blank-waiting+blanking;
			int nsteps = dt/step;
			if(steps < nsteps){
				steps++;
				return ButtonResult::ButtonPressing;
			}
		}

		return ButtonResult::None;
	}
};


class HalfBridge{
	mcpwm_unit_t unit;
	mcpwm_timer_t timer;
	int pin_a, pin_b;
	public:
	HalfBridge(mcpwm_unit_t unit, mcpwm_timer_t timer): unit(unit), timer(timer){};
	void setup(int pin_a, int pin_b, int freq_hz){
		ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0A, pin_a));
		ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0B, pin_b));
		this->pin_a = pin_a;
		this->pin_b = pin_b;
		    mcpwm_config_t cfg;
	    cfg.frequency = freq_hz;
	    cfg.cmpr_a = 0;
	    cfg.cmpr_b = 0;
	    cfg.duty_mode = MCPWM_HAL_GENERATOR_MODE_FORCE_LOW;//MCPWM_DUTY_MODE_0;
	    cfg.counter_mode = MCPWM_UP_COUNTER;
		ESP_ERROR_CHECK(mcpwm_init(unit, timer, &cfg));
	}
	
	void dead_time(int dt_us){
		ESP_ERROR_CHECK(mcpwm_deadtime_enable(unit, timer, MCPWM_BYPASS_FED, dt_us*10, dt_us*10));
	}

	// MCPWM_HAL_GENERATOR_MODE_FORCE_LOW, MCPWM_DUTY_MODE_0
	void duty_mode(mcpwm_duty_type_t t){
		ESP_ERROR_CHECK(mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, t));
		ESP_ERROR_CHECK(mcpwm_set_duty_type(unit, timer, MCPWM_OPR_B, t));
	}
	
	void pwm_mode(){
		mcpwm_start(unit,timer);
		duty_mode(MCPWM_DUTY_MODE_0);
	}
	
	void onoff_mode(){
		mcpwm_stop(unit,timer);
		pinMode(pin_a, OUTPUT);
		pinMode(pin_b, OUTPUT);
	}

	void set_a(float f){
		ESP_ERROR_CHECK(mcpwm_set_duty(unit, timer, MCPWM_OPR_A, f*100));
	}
	
	void set_b(float f){
		ESP_ERROR_CHECK(mcpwm_set_duty(unit, timer, MCPWM_OPR_B, f*100));
	}

	void on_a(){
		//ESP_ERROR_CHECK(mcpwm_set_signal_high(unit, timer, MCPWM_OPR_A));
		digitalWrite(pin_a, HIGH);
	}
	
	void on_b(){
		//ESP_ERROR_CHECK(mcpwm_set_signal_high(unit, timer, MCPWM_OPR_B));
		digitalWrite(pin_b, HIGH);
	}

	void off_a(){
		//ESP_ERROR_CHECK(mcpwm_set_signal_low(unit, timer, MCPWM_OPR_A));
		digitalWrite(pin_a, LOW);
	}
	
	void off_b(){
		//ESP_ERROR_CHECK(mcpwm_set_signal_low(unit, timer, MCPWM_OPR_B));
		digitalWrite(pin_b, LOW);
	}
	
	void set_frequency(int freq){
		ESP_ERROR_CHECK(mcpwm_set_frequency(unit, timer, freq));
	}

	void pulse(int usecs, int deadtime=30){
		off_b();
		delayMicroseconds(deadtime);
		on_a();
		delayMicroseconds(usecs);
		off_a();
		delayMicroseconds(deadtime);
		on_b();
	}
};

HalfBridge hb(MCPWM_UNIT_0, MCPWM_TIMER_0);

Button Left,Stop,Right;
#define LEFT 14
#define STOP 13
#define RIGHT 12

void setup(){
	strip.Begin();
 	strip.ClearTo(RgbColor(255,255,255));
	strip.Show();
	hb.setup(4, 5, 25000);
	Serial.begin(9600);
	//hb.pwm_mode();
	//hb.dead_time(30);
	pinMode(LEFT, INPUT_PULLUP);
	pinMode(STOP, INPUT_PULLUP);
	pinMode(RIGHT, INPUT_PULLUP);
}

// 10us on time
// 3.5us dead time

void loop(){
	int deadt=30;
	static int freq = 25000;
	bool changed = false;
	static bool on = true;
	switch(Left.feed(digitalRead(LEFT))){
		case ButtonPressed:
			freq += 10;
			changed = true;
		break;
		case ButtonPressing:
			freq += 10;
			changed = true;
		break;
		default:
		break;
	}
	
	switch(Stop.feed(digitalRead(STOP))){
		case ButtonPressed:
			on = !on;
			changed=true;
		break;
		default:
		break;
	}
	
	switch(Right.feed(digitalRead(RIGHT))){
		case ButtonPressed:
			freq -= 10;
			changed = true;
		break;
		case ButtonPressing:
			freq -= 10;
			changed = true;
		break;
		default:
		break;
	}
	
	if(changed){
		Serial.print("Changed\r\r");
		if(on){
			hb.set_frequency(freq);
			hb.pwm_mode();
		}
		else{
			hb.onoff_mode();
			hb.off_a();
			hb.on_b();
		}
	}
	
	/*
	if (Serial.available() > 0) {
		if(Serial.read() == 'p'){
			hb.pulse(45);
			Serial.write("Pulsed\r\n");
		}
	}*/
}
