#ifndef CDEBOUNCE_H_DEFINED
#define CDEBOUNCE_H_DEFINED
// ======================================================================
// input debouncing
// usage...
//   void cb_func(void){ // define call-back function
//     Serial.println("called."); 
//   }
//   void setup(){
//     CDebounce cdeb_pin_00(0, cb_func, FALLING); // debounce PIN #0, callback to cb_func
//   }
//   void loop(){
//     cdeb_pin_00.check(); // check pin event and make function call
//   }
// ======================================================================
class CDebounce{
  private: 
    int pin_no; // arduino pin number for the pin
    int pin_state; // pin state, HIGH or LOW
    bool pin_triggered; // if pin is triggered or not
    unsigned long last_trig_time; // last triggered time
    const unsigned long debounce_ms = 100; // debounce time period in ms
    int event_mode = CHANGE; // event mode
    void (*callback_func)(void) = nullptr; // pointer to callback function
    CDebounce(){};

    bool (*is_event_active)(bool is_trigger, int pin_state) = nullptr; // pointer to event detector 
    static bool is_event_low(    bool trg, int st){ return (st == LOW); };
    static bool is_event_change( bool trg, int st){ return (trg); };
    static bool is_event_rising( bool trg, int st){ return (trg && (st == HIGH)); };
    static bool is_event_falling(bool trg, int st){ return (trg && (st == LOW)); };

  public:
    // constructors
    CDebounce(
      const int new_pin_no, 
      void (*new_callback_func)(void), 
      const int new_mode, // LOW, CHANGE, RISING, FALLING
      const unsigned long new_debounce_ms = 100
    ): pin_no(new_pin_no), 
       pin_state(digitalRead(new_pin_no)), 
       pin_triggered(false), 
       last_trig_time(millis()), 
       event_mode(new_mode), 
       callback_func(new_callback_func)
    {
      switch(event_mode){
        case LOW:     is_event_active = is_event_low;     break;
        case CHANGE:  is_event_active = is_event_change;  break;
        case RISING:  is_event_active = is_event_rising;  break;
        case FALLING: is_event_active = is_event_falling; break;
        default:      is_event_active = nullptr;
      }
    };
    
    // pin event handler
    void check(void){
      // ignore any pin event while debouncing
      if((millis() - this->last_trig_time) < this->debounce_ms){
        this->pin_triggered = false;
        return;
      }

      // check current input pin state
      if(digitalRead(this->pin_no) == LOW){
        if(this->pin_state == HIGH){
          this->pin_triggered = true;
          this->pin_state = LOW;
          this->last_trig_time = millis();
        }
        else{
          this->pin_triggered = false;
        }
      }else{ // if the pin is HIGH
        if(this->pin_state == LOW){
          this->pin_triggered = true;
          this->pin_state = HIGH;
          this->last_trig_time = millis();
        }
        else{
          this->pin_triggered = false;
        }
      }

      // execute callback function
      if( is_event_active && (*is_event_active)(this->pin_triggered, this->pin_state) ){
        (*this->callback_func)();
      }
    }

    // read state
    int read(void){
      return this->pin_state;
    }

    // read trigger event
    bool is_triggered(void){
      return this->pin_triggered;
    }
};

#endif // CDEBOUNCE_H_DEFINED
