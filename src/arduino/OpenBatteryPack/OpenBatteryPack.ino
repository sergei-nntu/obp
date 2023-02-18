#include <EEPROM.h>

#include <LowPower.h>

// BATTERY CHARGE CONTROLLER FUNCTIONS

#define OBP_MAX_CAPACITY 10000 // mAh

int obp_voltage = 0;      // in mV
int obp_current_draw = 0; // in mA
int obp_soc = 2000;       // in mAh

// BATTERY PACK HARDWARE PARAMETERS

#define OBP_MAX_V       12600 // mV
#define OBP_MIN_V       9000  // mV

#define OBP_DIVISOR_R1  220
#define OBP_DIVISOR_R2  470

#define OBP_REF_V       5000
#define OBP_ADC_MAX     1024

#define OBP_V_PIN       A0
#define OBP_I_PIN       A1

#define OBP_STATUS_UNKNOWN        0x00
#define OBP_STATUS_IDLE           0x01
#define OBP_STATUS_DISCHARGING    0x02
#define OBP_STATUS_CHARGING       0x03
#define OBP_STATUS_FULLY_CHARGED  0x04

#define OBP_FSM_STATE_WAITING_FOR_ZERO  0x01
#define OBP_FSM_STATE_ZERO_CURRENT      0x02
#define OBP_FSM_STATE_NON_ZERO_CURRENT  0x03

#define OBP_AFTER_ZERO_ITERATIONS       10  // Number of 8 second intervals to check the voltage drop at non-zero current 

#define OBP_MAX_SENSED_CURRENT  20000   // mA
#define OBP_MIN_SENSED_CURRENT  -20000  // mA
#define OBP_NEAR_ZERO_CURRENT   250 // mA
#define OBP_SIGNIFICANT_CURRENT 1000 // mA
#define OBP_MAX_CURRENT_VOLTAGE 4500 // mV
#define OBP_MIN_CURRENT_VOLTAGE 500 // mV

#define OBP_EEPROM_POS_CURRENT_I_ADDR         0*sizeof(int)
#define OBP_EEPROM_POS_CURRENT_DELTA_U_ADDR   1*sizeof(int)
#define OBP_EEPROM_NEG_CURRENT_I_ADDR         2*sizeof(int)
#define OBP_EEPROM_NEG_CURRENT_DELTA_U_ADDR   3*sizeof(int)

// Corresponds to WAITING_FOR_ZERO state of FSM
unsigned char after_zero_iterations = OBP_AFTER_ZERO_ITERATIONS+1;
unsigned char obp_fsm_state = OBP_FSM_STATE_WAITING_FOR_ZERO;
int last_zero_current_voltage = 0;

unsigned char obp_status = OBP_STATUS_IDLE;

bool obp_flag_low_voltage = false;
bool obp_flag_high_voltage = false;
bool obp_flag_overheat = false;
bool obp_flag_short_circuit= false;

// OPEN SERIAL PERIPHERAL FUNCTIONS

#define OSP_MSG_DEV_INDEX     2
#define OSP_MSG_CMD_INDEX     3
#define OSP_BYTE_PARAM_INDEX      4
#define OSP_INT_PARAM_MSB_INDEX   5
#define OSP_INT_PARAM_LSB_INDEX   4 

#define OSP_DEV_GENERIC       0 
#define OSP_DEV_ORM           1 // Open Robotic Manipulator
#define OSP_DEV_OBP           2 // Open Battery Pack 
#define OSP_DEV_O2D           3 // Open Differential Drive

#define OSP_CMD_REQ_DEV_TYPE  1

#define OSP_OBP_CMD_REQ_SOC       0x01
#define OSP_OBP_CMD_REQ_VOLTAGE   0x02
#define OSP_OBP_CMD_REQ_CURRENT   0x03
#define OSP_OBP_CMD_REQ_STATUS    0x04

#define OSP_INFO_DEV_TYPE         0x11
#define OSP_OBP_INFO_SOC          0x11
#define OSP_OBP_INFO_VOLTAGE      0x12
#define OSP_OBP_INFO_CURRENT      0x13
#define OSP_OBP_INFO_STATUS       0x14

#define OSP_DEV_CURRENT           OSP_DEV_OBP

#define OSP_COMMAND_LENGTH        10
#define OSP_BUFFER_SIZE           10

char            osp_output_buffer[OSP_BUFFER_SIZE];
unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
int             osp_ptr=0;

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

// HANDING OSP GENERIC COMMANDS

void osb_info_dev_type(){
  osp_prepare_output_buffer();
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_GENERIC;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_INFO_DEV_TYPE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = OSP_DEV_CURRENT; // Returning the current device type as a parameter
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void osp_handle_generic_command(){
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_CMD_REQ_DEV_TYPE) {
    // HERE WE MUST RESPONSE WITH A DEV TYPE
    osb_info_dev_type();
  }
}

void osp_prepare_output_buffer(){
  for(int i=0;i<OSP_BUFFER_SIZE;i++) {
    osp_output_buffer[i] = osp_command_template[i];
  }
}

// HANDLING OBP COMMANDS
void osp_opb_send_info_soc(){
  osp_prepare_output_buffer();
    
  unsigned int soc_int = obp_soc;
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_SOC;
  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = soc_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = soc_int >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void osp_opb_send_info_voltage(){
  osp_prepare_output_buffer();
  unsigned int voltage_int = obp_voltage ; // Voltage in mV

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_VOLTAGE;
  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = voltage_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = voltage_int >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);  
}

void osp_opb_send_info_current(){
  osp_prepare_output_buffer();
  int  current_int = obp_current_draw; // mA 
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_CURRENT;

  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = current_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = current_int >> 8;
   
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);   
}

void osp_opb_send_info_status(){
  
}

void osp_handle_obp_command(){
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_OBP_CMD_REQ_SOC) {
    // RESPONSE WITH A CURRENT SOC STATE
    osp_opb_send_info_soc();
  }
  if (cmd == OSP_OBP_CMD_REQ_VOLTAGE) {
    // RESPONSE WITH A CURRENT VOLTAGE LEVEL
    osp_opb_send_info_voltage();
  }
  if (cmd == OSP_OBP_CMD_REQ_CURRENT) {
    // RESPONSE WITH A CURRENT CURRENT DRAW
    osp_opb_send_info_current();
  }
  if (cmd == OSP_OBP_CMD_REQ_STATUS) {
    // RESPONSE WITH A CURRENT BATTERY STATUS
    osp_opb_send_info_status();
  }
}

void osp_handle_command(){
  int dev = osp_input_buffer[OSP_MSG_DEV_INDEX];
  
  if (dev == OSP_DEV_GENERIC) {
    osp_handle_generic_command();
  } else if (dev == OSP_DEV_CURRENT) {
    if (dev == OSP_DEV_OBP) {
      osp_handle_obp_command();
    }
  } else {
    // ERROR CONDITION. THE CLIENT MUST NOT SEND THE COMMAND WHICH ARE NOT FOR EITHER GENERIC OR CURRENT DEV TYPE
    // DOING NOTHING FOR NOW
  }
}

void setup_osp(){
  Serial.begin(115200);
  Serial.setTimeout(0.01);  
}

void loop_osp(){
  if(Serial.available()){
    char b = Serial.read();
    if(osp_command_template[osp_ptr]==b || osp_command_template[osp_ptr]==0){
      osp_input_buffer[osp_ptr] = b;
      osp_ptr++; 
      if(osp_ptr==OSP_COMMAND_LENGTH){
        osp_handle_command();
        osp_ptr = 0;
      }
    } else {
      osp_ptr = 0;
    }
  }
}

// OPB Logic
int pos_i =0;
int pos_delta_u = 0;
int neg_i=0;
int neg_delta_u = 0;

bool obp_i_delta_u_EEPROM_stored = false; 

void obp_store_i_deltaU_EEPROM(){
  if(pos_i>0 && neg_i<0){
    if (obp_i_delta_u_EEPROM_stored==false) {
      // Write the values to EEPROM
      // 1. Get the values currently written to EEPROM
      int loaded_pos_i = 0;
      int loaded_pos_delta_U = 0;
      int loaded_neg_i = 0;
      int loaded_neg_delta_U = 0; 

      EEPROM.get(OBP_EEPROM_POS_CURRENT_I_ADDR, loaded_pos_i);
      EEPROM.get(OBP_EEPROM_POS_CURRENT_DELTA_U_ADDR, loaded_pos_delta_U);

      EEPROM.get(OBP_EEPROM_NEG_CURRENT_I_ADDR, loaded_neg_i);
      EEPROM.get(OBP_EEPROM_NEG_CURRENT_DELTA_U_ADDR, loaded_neg_delta_U);

      // 2. Compare the values from EEPROM with the current values, and write the changes 
      if(loaded_pos_i != pos_i){
        EEPROM.put(OBP_EEPROM_POS_CURRENT_I_ADDR, pos_i);
      }
      if(loaded_pos_delta_U != pos_delta_u) {
        EEPROM.put(OBP_EEPROM_POS_CURRENT_DELTA_U_ADDR, pos_delta_u);
      }

      if(loaded_neg_i != neg_i) {
        EEPROM.put(OBP_EEPROM_NEG_CURRENT_I_ADDR, neg_i);        
      }   

      if(loaded_neg_delta_U != neg_delta_u) {
        EEPROM.put(OBP_EEPROM_NEG_CURRENT_DELTA_U_ADDR, neg_delta_u);  
      }   

      obp_i_delta_u_EEPROM_stored=true;
    }
  }
}

void obp_load_i_deltaU_EEPROM(){
  int loaded_pos_i = 0;
  int loaded_pos_delta_U = 0;
  int loaded_neg_i = 0;
  int loaded_neg_delta_U = 0;  

  EEPROM.get(OBP_EEPROM_POS_CURRENT_I_ADDR, loaded_pos_i);
  EEPROM.get(OBP_EEPROM_POS_CURRENT_DELTA_U_ADDR, loaded_pos_delta_U);

  EEPROM.get(OBP_EEPROM_NEG_CURRENT_I_ADDR, loaded_neg_i);
  EEPROM.get(OBP_EEPROM_NEG_CURRENT_DELTA_U_ADDR, loaded_neg_delta_U);


  if(loaded_pos_i>0 && loaded_neg_i<0){
    // Load only if the values are respectively below and above zero
    pos_i = loaded_pos_i;
    pos_delta_u = loaded_pos_delta_U;
    neg_i = loaded_neg_i;
    neg_delta_u = loaded_neg_delta_U;
  }
}

void obp_save_i_delta_u(int i, int delta_u){
  if(i>=0){
    pos_i = i;
    pos_delta_u = delta_u;
  } else {
    neg_i = i;
    neg_delta_u = delta_u;
  }  
}

long int obp_get_delta_u(long int i){
    if(i>=0){
      if (pos_i ==0){
        return 0;
      }      
      return i * pos_delta_u / pos_i;
    } else {
      if(neg_i ==0) {
        return 0;
      }      
      return i * neg_delta_u / neg_i;
    }
    return 0;
}

void obp_control_thread(){
  // OPTION 1. MVP
  // Just read the current V and I values
  //  - they may also be converted to the battery chanrge level using some basic conversion table/formula
  long int voltage_adc = analogRead(OBP_V_PIN);
  obp_voltage = (OBP_REF_V * voltage_adc / OBP_ADC_MAX) * (OBP_DIVISOR_R1+OBP_DIVISOR_R2) / (OBP_DIVISOR_R1); 

  long int current_adc = analogRead(OBP_I_PIN);
  long int current_adc_v = current_adc * OBP_REF_V / OBP_ADC_MAX;
  
  obp_current_draw = OBP_MIN_SENSED_CURRENT + (current_adc_v - OBP_MIN_CURRENT_VOLTAGE) * ((long)OBP_MAX_SENSED_CURRENT-OBP_MIN_SENSED_CURRENT) / (OBP_MAX_CURRENT_VOLTAGE - OBP_MIN_CURRENT_VOLTAGE);
  //obp_current_draw = -current_adc_v;
  // RECORD THE DELTA U FOR THE CERTAIN VALUE OF I

  if (obp_current_draw>=-OBP_NEAR_ZERO_CURRENT && obp_current_draw<=OBP_NEAR_ZERO_CURRENT){
    // Record U and nullify N
    after_zero_iterations = 0;
    last_zero_current_voltage = obp_voltage;
    obp_fsm_state = OBP_FSM_STATE_ZERO_CURRENT;
  } else {
    if(obp_fsm_state == OBP_FSM_STATE_ZERO_CURRENT || obp_fsm_state == OBP_FSM_STATE_NON_ZERO_CURRENT) {
      obp_fsm_state = OBP_FSM_STATE_NON_ZERO_CURRENT;
      after_zero_iterations++;
      if(after_zero_iterations > OBP_AFTER_ZERO_ITERATIONS){
        obp_fsm_state = OBP_FSM_STATE_WAITING_FOR_ZERO;
        obp_store_i_deltaU_EEPROM(); // Once the period of measuring of the deltas is ended - store the results to EEPROM
      }
    }
    if(obp_fsm_state == OBP_FSM_STATE_NON_ZERO_CURRENT) {
      int deltaU = obp_voltage - last_zero_current_voltage;
      obp_save_i_delta_u(obp_current_draw, deltaU);
    }
  }

  // ESTIMATE THE SOC
  long int deltaUExpected = obp_get_delta_u(obp_current_draw);
  long int noLoadU = obp_voltage - deltaUExpected;

  obp_soc = OBP_MAX_CAPACITY * (noLoadU - OBP_MIN_V) / (OBP_MAX_V - OBP_MIN_V);
  if (obp_soc<0){
    obp_soc = 0;
  }
  if(obp_soc > OBP_MAX_CAPACITY){
    obp_soc = OBP_MAX_CAPACITY;    
  }
}

void setup() {
  // put your setup code here, to run once:
  setup_osp();
  obp_load_i_deltaU_EEPROM();
}

void loop() {
  // put your main code here, to run repeatedly:
  loop_osp();
  obp_control_thread();
  osp_opb_send_info_soc();  
  osp_opb_send_info_current();
  osp_opb_send_info_voltage();
  osb_info_dev_type();
  delay(100);
  
  LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
}
