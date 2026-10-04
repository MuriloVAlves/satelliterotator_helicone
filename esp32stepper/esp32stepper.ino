/********** Include **********/
#include <Wire.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>

/********** Setup **********/
// WiFi configurations
// #define WIFI_STATIC //uncomment this line for static address
#define SSID "SSID"
#define PASSWORD "PASSWORD"
String command_buff;
String response = "";
uint8_t bytes_read, bytes_sent;
static uint16_t acum_read, acum_sent;
#ifdef WIFI_STATIC
IPAddress local_IP(192, 168, 0, 52);
IPAddress gateway(192, 168, 1, 5);
IPAddress subnet(255, 255, 255, 0);
#endif
// TCP server listening on port 4533
WiFiServer tcpServer(4533);
#define TCP_HANDLE_PERIOD 500

// LCD I2C configurations
#define LCD_WIDTH 16
#define LCD_HEIGHT 2
#define LCD_ADDRESS 0x27
#define SDA_PIN 21
#define SCL_PIN 22
LiquidCrystal_I2C display(LCD_ADDRESS, LCD_WIDTH, LCD_HEIGHT);
// Custom screen infos
enum Screen_Types { NOT_CON,
                    IP_INFO,
                    AZ_EL_INFO,
                    DIFF_INFO,
                    COMM_INFO };
Screen_Types screen_status = NOT_CON;
#define REFRESH_INTERVAL  500
#define UPDATE_INTERVAL   5000
#define ERROR_INTERVAL    1000
// Queues to handle external requests
static const uint8_t connectedQueueLen = 5;
static QueueHandle_t connectedQueue;

// Rotator state
#define LIMIT_MIN  -175.0 // Deixando 5 graus de margem de manobra
#define LIMIT_MAX  535.0
double targetAz = 0, targetEl = 0;
double actualAz = 360, actualEl = 0; // actualAz: 360 -> 1.5 turns down and up (-180 to 540)
double delta_az = 0;
double delta_el = 0;
bool azMove = false, elMove = false;
String inputCommand;

// DRV8825 variables
// Shared M0, M1, M2, RST and DIR pins
#define M0_PIN        15
#define M1_PIN        2
#define M2_PIN        4
#define RSTN_PIN      16
#define DIR_PIN       17
#define STEP_AZ_PIN   5
#define STEP_EL_PIN   18
#define FAULT_AZ_PIN  13
#define FAULT_EL_PIN  12
// Define gear ratio
#define GEAR_RATIO    2
#define MTR_FULL_STP  1.8
#define REAL_FULL_STP MTR_FULL_STP/GEAR_RATIO
double step_angles[6] = {REAL_FULL_STP,REAL_FULL_STP/2,REAL_FULL_STP/4,REAL_FULL_STP/8,REAL_FULL_STP/16,REAL_FULL_STP/32};
#define MIN_STEP_IDX (sizeof(step_angles)/sizeof(double))-1
#define SMALLEST_STP  step_angles[MIN_STEP_IDX]
// Define state of "DIR" pin for motors
#define AZ_POS_DIR true
#define AZ_NEG_DIR false
#define EL_POS_DIR true
#define EL_NEG_DIR false
// Bools to register motor directions and steps
bool step_az_dir = AZ_POS_DIR;
bool step_el_dir = EL_POS_DIR;
bool step_az_stp = false; // last step pin output
bool step_el_stp = false; // last step pin output
// Start motors on smallest step
uint8_t  step_az_micro = MIN_STEP_IDX; // microstep index
uint8_t  step_el_micro = MIN_STEP_IDX; // microstep index
const uint8_t steps_until_transition = 2; // mantain acceleration of motor
const uint8_t steps_factor = 3; // slow down factor
uint8_t step_az_accel = 0, step_el_accel = 0; // acceleration steps until transition
// Step frequencies
#define STP_SWITCH_PERIOD 10 //ms

// Alpha-Beta-Gamma Filters
#define ALPHA_AZ  0.937
#define BETA_AZ   0.298
#define GAMMA_AZ  0.12
#define ALPHA_EL  1.0
#define BETA_EL   0.253
#define GAMMA_EL  0.122
#define PREDICT_PERIOD 100
static double pos_est_az, vel_est_az, accel_est_az = 0;
static double pos_est_el, vel_est_el, accel_est_el = 0;
// Queues to handle external requests
static const uint8_t receivedGuessLen = 5;
static QueueHandle_t receivedGuess;



/********** Functions **********/
// LCD String handling
String str_stuffing(String text) {
  String new_text = text;
  if (text.length() < 16) {
    // add spaces to the end of text
    bool stuffing = 1;
    while (stuffing) {
      new_text = new_text + " ";
      if (new_text.length() >= 16) {
        stuffing = 0;
      }
    }
  }
  return new_text;
}

String str_centering(String text) {
  String new_text = text;
  if (text.length() < 16) {
    // add spaces to the end of text
    bool stuffing = 1;
    bool left_right = 1;  // 0 to left; 1 to rigth;
    while (stuffing) {
      if (left_right) {
        new_text = new_text + " ";
        left_right = 0;
      } else {
        new_text = " " + new_text;
        left_right = 1;
      }

      if (new_text.length() >= 16) {
        stuffing = 0;
      }
    }
  }
  return new_text;
}

void updateDisplaySttMchn(){
  // Create a carousel of debug informations
  switch (screen_status) {
    case AZ_EL_INFO:
      screen_status = DIFF_INFO;
      break;
    case DIFF_INFO:
      screen_status = COMM_INFO;
      break;
    case COMM_INFO:
      acum_read = 0;
      acum_sent = 0;
      screen_status = AZ_EL_INFO;
      break;
    case NOT_CON:
      screen_status = IP_INFO;
    case IP_INFO:
      screen_status = NOT_CON;
  }
}

void update_screen() {
  switch (screen_status) {
    case NOT_CON:
      display.setCursor(0, 0);
      display.print(" NO CONNECTION! ");
      display.setCursor(0, 1);
      display.print("   Waiting...   ");
      break;
    case IP_INFO:
      display.setCursor(0, 0);
      display.print("   IP Address   ");
      display.setCursor(0, 1);
      display.print(str_centering(WiFi.localIP().toString()));
      break;
    case AZ_EL_INFO:
      display.setCursor(0, 0);
      display.print(str_centering("Az: "+String(actualAz,6)));
      display.setCursor(0, 1);
      display.print(str_centering("El: "+String(actualEl,6)));
      break;
    case DIFF_INFO:
      display.setCursor(0, 0);
      display.print(str_centering("Dif Az: "+String(delta_az,6)));
      display.setCursor(0, 1);
      display.print(str_centering("Dif El: "+String(delta_el,6)));
      break;
    case COMM_INFO:
      display.setCursor(0, 0);
      display.print(str_centering("VAz:"+String(step_angles[step_az_micro],3)+" RX:"+String(acum_read)));
      display.setCursor(0,1);
      display.print(str_centering("VEl:"+String(step_angles[step_el_micro],3)+" TX:"+String(acum_sent)));
      break;
  }
}

void handle_display(void *parameters){
  unsigned long lastRefresh = 0;
  unsigned long lastUpdateMs = 0;
  unsigned long lastNoConUpdate = 0;
  bool satdumpConnected = false;
  bool errorScreen = false;
  // Handle screen switching intervals while connected
  while (1){
    // Check queue
    if (xQueueReceive(connectedQueue, (void *)&satdumpConnected, 0) == pdTRUE){
      // Compare received status with error status
      if (satdumpConnected && errorScreen){
        // Return to normal telemetry
        errorScreen = false;
        screen_status = AZ_EL_INFO;
      }
      if ((!satdumpConnected)&&(!errorScreen)){
        // Enter error state
        errorScreen = true;
        screen_status = NOT_CON;
      }
    }

    if(satdumpConnected){
      if (millis() - lastUpdateMs >= UPDATE_INTERVAL) {
        updateDisplaySttMchn();
        lastUpdateMs = millis();
      }

      if (millis() - lastRefresh >= REFRESH_INTERVAL) {
        update_screen();
        lastRefresh = millis();
      }

    }
    // DISCONNECTED DISPLAY BLINKING
    else {
      if (millis() - lastNoConUpdate >= ERROR_INTERVAL) {
        lastNoConUpdate = millis();
        update_screen();
      }
    }
    vTaskDelay(REFRESH_INTERVAL/portTICK_PERIOD_MS);
  }
}


// Motor functions
void step_accel(uint8_t *accel, uint8_t *micro) {
    if (*accel == 0) {
        *accel = steps_until_transition;
        if (*micro > 0) {
            *micro -= 1;
        }
    } else {
        *accel -= 1;
    }
}

void step_deaccel(uint8_t *micro) {
    if (*micro < MIN_STEP_IDX) {
        *micro += 1;
    }
}

void start_inversion(uint8_t *micro, bool *dir) {
    if (*micro == MIN_STEP_IDX) {
        *dir = !(*dir);
    } else {
        step_deaccel(micro);
    }
}

void handle_steppers(){
    // AZIMUTH
    handle_axis(&azMove, targetAz, actualAz, &delta_az,
                &step_az_dir, AZ_POS_DIR, AZ_NEG_DIR,
                &step_az_micro, &step_az_accel);

    // ELEVATION
    handle_axis(&elMove, targetEl, actualEl, &delta_el,
                &step_el_dir, EL_POS_DIR, EL_NEG_DIR,
                &step_el_micro, &step_el_accel);
}

void handle_axis(bool *move, double target, double actual, double *delta,
                 bool *step_dir, int pos_dir, int neg_dir,
                 uint8_t *step_micro, uint8_t *stp_accel) {
    // Check if step is enabled
    if (*move) {
        *delta = target - actual;
        // Disable step if there is no more step to have
        if (abs(*delta) < SMALLEST_STP) {
            *move = false;
        } else { // If movement is still necessary
            if (*delta > 0) {
                if (*step_dir == pos_dir) {
                    // Handle decel vs acceleration
                    if (abs(*delta) < steps_factor * step_angles[*step_micro]) {
                        step_deaccel(step_micro);
                    } else {
                        step_accel(stp_accel, step_micro);
                    }
                } else {
                    start_inversion(step_micro, step_dir);
                }
            } else {
                if (*step_dir == neg_dir) {
                    // Handle decel vs acceleration
                    if (abs(*delta) < steps_factor * step_angles[*step_micro]) {
                        step_deaccel(step_micro);
                    } else {
                        step_accel(stp_accel, step_micro);
                    }
                } else {
                    start_inversion(step_micro, step_dir);
                }
            }
        }
    }
}

void step_motors(void *parameters){
  while (1){
    // TODO: HANDLE FAULTY CURRENT
    if (!(azMove || elMove)){
      // if movement is not necessary
      return;
    }
    // Serial.println("Handled step");
    // If it's time for one more step
      handle_steppers();
      if(azMove){
        //Write pin control
        step_az_stp = !step_az_stp;
        digitalWrite(M0_PIN,bitRead(step_az_micro, 0)); // M0
        digitalWrite(M1_PIN,bitRead(step_az_micro, 1)); // M1
        digitalWrite(M2_PIN,bitRead(step_az_micro, 2)); // M2
        digitalWrite(DIR_PIN,step_az_dir);
        digitalWrite(RSTN_PIN,HIGH);
        digitalWrite(STEP_AZ_PIN,step_az_stp); // step
        if(step_az_stp){
          if(step_az_dir == AZ_POS_DIR){
            actualAz += step_angles[step_az_micro];
          }
          else{
            actualAz -= step_angles[step_az_micro];
          }
        }
      }
      if(elMove){
        //Write pin control
        step_el_stp = !step_el_stp;
        digitalWrite(M0_PIN,bitRead(step_el_micro, 0)); // M0
        digitalWrite(M1_PIN,bitRead(step_el_micro, 1)); // M1
        digitalWrite(M2_PIN,bitRead(step_el_micro, 2)); // M2
        digitalWrite(DIR_PIN,step_el_dir);
        digitalWrite(RSTN_PIN,HIGH);
        digitalWrite(STEP_EL_PIN,step_el_stp); // step
        if(step_el_stp){
          if(step_el_dir == EL_POS_DIR){
            actualEl += step_angles[step_el_micro];
          }
          else{
            actualEl -= step_angles[step_el_micro];
          }
        }
      }
    vTaskDelay(STP_SWITCH_PERIOD/portTICK_PERIOD_MS);
  }
}


// Alfa-Beta-Gamma Filter
void update_filter(void *parameters){
  bool first_run = false;
  double now = 0, delta = 0, // time vars
        pos_pred = 0, vel_pred = 0, accel_pred = 0, // prediction vars
        pos_real = 0; // prediction vars

  now = time.time()
  delta = now - self.last_updt
  
  if self.first_run:
      self.pos = new_val
      self.last_updt = now
      self.first_run = False
      return

  if delta <= 0: 
      delta = 0.001

  // 1. PASSO DE PREDIÇÃO (Onde o estado deveria estar baseado no tempo)
  pos_pred = self.pos + (self.vel * delta) + (0.5 * self.accel * (delta ** 2))
  vel_pred = self.vel + (self.accel * delta)
  accel_pred = self.accel

  // 2. CÁLCULO DO ERRO (Resíduo da medição real vs predita)
  r = new_val - pos_pred

  // 3. PASSO DE ATUALIZAÇÃO (Correção usando os ganhos alfa, beta, gamma)
  self.pos   = pos_pred   + self.alfa * r
  self.vel   = vel_pred   + (self.beta / delta) * r
  self.accel = accel_pred + (2 * self.gamma / (delta ** 2)) * r

  self.last_updt = now
}


// TCP functions
String handle_command(){
  String response = "";
  if (command_buff[0] == 'p'){
      double reportAz = actualAz;
      while (reportAz >= 360.0) reportAz -= 360.0;
      while (reportAz < 0.0)    reportAz += 360.0;

      response += String(reportAz,4);
      response += " ";
      response += String(actualEl,4);
  }
  else if (command_buff[0] == 'P'){
    // Trim command header
    String update_values = String(command_buff);
    int command_split = update_values.indexOf(' '); // Get first space
    // Get string with values
    update_values = update_values.substring(command_split + 1);
    // Get target azimuth and elevation
    command_split = update_values.indexOf(' ');
    // Update targets
    double rawTargetAz = update_values.substring(0,command_split).toDouble();
    targetEl = update_values.substring(command_split + 1).toDouble();
    // Serial.println("Set targets: "+String(targetAz,3)+" "+String(targetEl,3));
    double option_current = rawTargetAz;
    double option_plus    = rawTargetAz + 360.0;
    double option_minus   = rawTargetAz - 360.0;

    double dist_curr  = abs(option_current - actualAz);
    double dist_plus  = abs(option_plus - actualAz);
    double dist_minus = abs(option_minus - actualAz);

    double bestTarget = option_current;
    double minDist = dist_curr;

    if (dist_plus < minDist) {
        minDist = dist_plus;
        bestTarget = option_plus;
    }
    if (dist_minus < minDist) {
        minDist = dist_minus;
        bestTarget = option_minus;
    }

    // histeresis:
    if (azMove) {
        double currentDelta = bestTarget - actualAz;
        if (step_az_dir == AZ_NEG_DIR && currentDelta > 180.0) {
            bestTarget = bestTarget - 360.0;
        } else if (step_az_dir == AZ_POS_DIR && currentDelta < -180.0) {
            bestTarget = bestTarget + 360.0;
        }
    }

    // Handles over rotation
    if (bestTarget > LIMIT_MAX) {
        bestTarget = option_minus;
    } else if (bestTarget < LIMIT_MIN) {
        bestTarget = option_plus;
    }

    targetAz = bestTarget;
    azMove = true;
    elMove = true;
    response = command_buff;
  }
  // return command
  return response;
}

void handle_tcp_request(void *parameters){
  // TCP CONNECTION MANAGEMENT (NON-BLOCKING)
  static WiFiClient activeClient;
  static bool clientConnected = false; //Memory to pass with queue method
  // Start the socket server
  tcpServer.begin();
  
  while(1){
    if (!activeClient) {
      // If no client is currently active, check for a incoming connection
      activeClient = tcpServer.available();
      if (activeClient) {
        Serial.println("New Client Connected.");
        clientConnected = true;
        if (xQueueSend(connectedQueue,(void *)&clientConnected, (TickType_t)1) != pdTRUE){
          Serial.println("Connected queue full...");
        }
      }
    }
    if (activeClient && !activeClient.connected()) {
      // Clean up when the client disconnects or if SatDump closes the socket voluntarily
      activeClient.stop();
      Serial.println("Client Disconnected.");
      clientConnected = false;
      if (xQueueSend(connectedQueue,(void *)&clientConnected, (TickType_t)1) != pdTRUE){
        Serial.println("Connected queue full...");
      }
    }
    // PROCESS ACTIVE CLIENT DATA
    if (activeClient && activeClient.connected()) {
    // If bytes are waiting in the buffer, process ONE command per loop cycle
      if (activeClient.available() > 0) {
        // Read until '\n' and strip hidden characters like '\r'
        command_buff = activeClient.readStringUntil('\n');
        command_buff.trim();
        command_buff.replace('\\',command_buff[1]); // Look4Sat extra char (bug?)

        if (command_buff.length() > 0) {
          bytes_read = command_buff.length();
          // Serial.println("Command: " + command_buff + " Bytes: " + String(bytes_read));

          // Process the rotctld/Easycomm command and reply immediately
          response = handle_command();
          activeClient.print(response);
          activeClient.flush();

          bytes_sent = response.length();
          // Serial.println("Response: " + response + " Bytes: " + String(bytes_sent));

          // Transfer rate
          acum_read += bytes_read;
          acum_sent += bytes_sent;
        }
        command_buff = "";
      }
    }
  vTaskDelay(TCP_HANDLE_PERIOD/portTICK_PERIOD_MS);
  }
}



/********** Arduino Loop **********/

void setup() {
  // setup pins
  // stepper
  pinMode(M0_PIN,OUTPUT);
  pinMode(M1_PIN,OUTPUT);
  pinMode(M2_PIN,OUTPUT);
  pinMode(RSTN_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_AZ_PIN,OUTPUT);
  pinMode(STEP_EL_PIN,OUTPUT);
  pinMode(FAULT_AZ_PIN,INPUT);
  pinMode(FAULT_EL_PIN,INPUT);
  // init serial
  Serial.begin(115200);

  // initialize LCD
  display.init();
  // clear display
  display.setCursor(0, 0);
  display.print("                ");
  display.setCursor(0, 1);
  display.print("                ");
  // turn on LCD backlight
  display.backlight();
  // Set init message
  display.setCursor(0, 0);
  display.print("Initializing... ");

  #ifdef WIFI_STATIC
    if (!WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("Static IP configuration failed!");
      display.setCursor(0, 1);
      display.println("STATIC IP FAIL!");
    } else {
      display.setCursor(0, 1);
      display.print("IP configured!");
    }

  #else
  // Connect to Wi-Fi network
  Serial.println("Trying to connect to Wi-Fi");
  display.setCursor(0, 1);
  display.print("                ");
  WiFi.begin(SSID, PASSWORD);
  char wait_downtime = 0;
  char loading_chars[4] = {'/','-','\\','|'};
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.setCursor(0, 1);
    display.print("Connecting...");
    display.setCursor(1,(LCD_WIDTH/2)-1);
    display.print(loading_chars[wait_downtime]);
    wait_downtime = (wait_downtime > 3)? 0 : wait_downtime + 1;
  }
  display.setCursor(0, 1);
  display.print("WiFi connected!");
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  #endif


  // Create Queues
  // LCD
  connectedQueue = xQueueCreate(connectedQueueLen, sizeof(bool));

}

void loop() {
  // MOTOR MANAGEMENT
  // Executed on every single loop cycle to ensure stepper motor pulses
  // step_motors();
}