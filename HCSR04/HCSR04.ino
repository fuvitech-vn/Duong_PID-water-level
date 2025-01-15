#include <PID_v1.h>

// Các định nghĩa của bạn từ trước
#define BT1 18
#define BT2 23
#define BT3 13
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int trig = 17;
const int echo = 16;
const char* ssid = "DACN";
const char* password = "12345678";
const char* mqttServer = "mqtt.fuvitech.vn";
const int mqttPort = 2883;
const char* publishTopic = "truong/mucnuoc"; // topic nhận giữ liệu mqtt
const char* subscribeTopic1 = "truong/mucnuocgioihan"; // topic lấy giữ liệu
const char* subscribeTopic2 = "truong/Chedo"; // topic lấy giữ liệu
int chieudai = 15; // chiều dài của bình
WiFiClient espClient; // tạo clinet kết nối wifi
PubSubClient client(espClient); // client để publish

int mucNuocGioiHan = 0; 
int Chedo = 0;
bool settingMode = false; // vào chế độ set mực nước
double Setpoint, Input, Output; // setpoint : mực nước giới hạn, input: mực nước,output: xung pwm
double Kp = 16, Ki = 0.4, Kd = 0;  // Điều chỉnh Kp, Ki, Kd theo yêu cầu hệ thống
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void callback(char* topic, byte* payload, unsigned int length) { // hàm nhận giữ liệu từ mqtt gửi xuống
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    if (String(topic) == subscribeTopic1 && Chedo == 1) { // nếu dữ liệu nhận được ở topic mucnuocgioihan và Chedo == 1
        mucNuocGioiHan = message.toInt();
        Serial.print("Mực nước giới hạn từ MQTT: ");
        Serial.println(mucNuocGioiHan);
    } else if (String(topic) == subscribeTopic2) {
        Chedo = message.toInt();
        Serial.print("Chế độ: ");
        Serial.println(Chedo);
        if (Chedo == 1) {
            settingMode = false;
        }
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(trig, OUTPUT); // chan tao xung cam bien hcsr04
    pinMode(echo, INPUT); // chan tin hieu
    pinMode(BT1, INPUT_PULLUP); // nút nhấn mode
    pinMode(BT2, INPUT_PULLUP); // nút tăng
    pinMode(BT3, INPUT_PULLUP); // nút giảm 
    pinMode(19, OUTPUT);  // Chân PWM cho đầu ra PID

    lcd.init(); // khởi động màn hình lcd
    lcd.backlight(); // bật đèn màn hình

    WiFi.begin(ssid, password);// kết nối wifi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");

    client.setServer(mqttServer, mqttPort); // kết nối server mqtt
    client.setCallback(callback); // nhận giữ liệu từ mqtt xuống esp32
    connectToMQTT();

    // Thiết lập Setpoint và cấu hình PID
    Setpoint = mucNuocGioiHan; 
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);  // Đặt giới hạn cho output PID (0-255 cho PWM)
}

void connectToMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
            client.subscribe(subscribeTopic1); // đăng ký topic 1 để nhận giữ liệu
            client.subscribe(subscribeTopic2); // đăng ký topic 2 để giữ liệu
        } else {
            Serial.print("failed with state ");
            Serial.println(client.state());
            delay(2000);
        }
    }
}

void loop() {
    if (!client.connected()) {
        connectToMQTT();
    }
    client.loop();

    if (Chedo == 0) {
        if (digitalRead(BT1) == LOW) {
            delay(200);
            settingMode = !settingMode;
            Serial.println(settingMode ? "Vào chế độ cài đặt mực nước giới hạn" : "Mực nước giới hạn được xác nhận");
            while (digitalRead(BT1) == LOW);
        }
        if (settingMode) {
            if (digitalRead(BT2) == LOW) {
                delay(200);
                mucNuocGioiHan++;
                Serial.print("Tăng mực nước giới hạn: ");
                Serial.println(mucNuocGioiHan);
                lcd.setCursor(0, 1);
                lcd.print("Gioi han: ");
                lcd.print(mucNuocGioiHan);
                lcd.print(" cm ");
                while (digitalRead(BT2) == LOW);
            }
            if (digitalRead(BT3) == LOW) {
                delay(200);
                mucNuocGioiHan--;
                Serial.print("Giảm mực nước giới hạn: ");
                Serial.println(mucNuocGioiHan);
                lcd.setCursor(0, 1);
                lcd.print("Gioi han: ");
                lcd.print(mucNuocGioiHan);
                lcd.print(" cm ");
                while (digitalRead(BT3) == LOW);
            }
        } else {
            measureAndDisplayDistance();
        }
    } else if (Chedo == 1) {
        measureAndDisplayDistance();
    }
}

void measureAndDisplayDistance() {
    unsigned long duration;
    int distance;

    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = chieudai - (int(duration / 2 / 29.412));

    Input = distance;          // Cập nhật giá trị khoảng cách cho PID
    Setpoint = mucNuocGioiHan; // Cập nhật setpoint cho PID
    myPID.Compute();           // Tính toán PID

    analogWrite(19, Output);   // Điều chỉnh PWM trên chân 19 dựa trên đầu ra PID

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Muc nuoc: ");
    lcd.print(distance);
    lcd.print(" cm");
    lcd.setCursor(0, 1);
    lcd.print("Gioi han: ");
    lcd.print(mucNuocGioiHan);
    lcd.print(" cm");

    char msg[50];
    sprintf(msg, "%d", distance);
    client.publish(publishTopic, msg);

    delay(500);
}