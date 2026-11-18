#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"

// Must match the GASES list on the website
const char* KEYS[] = {"CO", "CO2", "NO2", "VOC", "PM25"};
const int N = 5;

String idToken = "";
unsigned long tokenTime = 0;

// Step 1: log in as the device and get a token (a temporary pass valid ~1 hour)
bool signIn() {
  WiFiClientSecure client;
  client.setInsecure();   // skips certificate checking: OK for a class demo, mention as a limitation
  HTTPClient http;
  http.begin(client, String("https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=") + API_KEY);
  http.addHeader("Content-Type", "application/json");

  JsonDocument body;
  body["email"] = String(DEVICE_ID) + "@monitor.example";
  body["password"] = DEVICE_PASSWORD;
  body["returnSecureToken"] = true;
  String payload;
  serializeJson(body, payload);

  int code = http.POST(payload);
  if (code != 200) {
    Serial.println(http.getString());
    http.end();
    return false;
  }
  JsonDocument resp;
  deserializeJson(resp, http.getString());
  idToken = resp["idToken"].as<String>();
  tokenTime = millis();
  http.end();
  return true;
}

// Time the reading was taken, in the format Firestore expects (UTC)
String isoTimeNow() {
  time_t now = time(nullptr);
  struct tm t;
  gmtime_r(&now, &t);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

// Firestore's REST format wants every value labelled with its type
void addGas(JsonObject gasFields, const char* key, float value, float baseline) {
  JsonObject f = gasFields[key]["mapValue"]["fields"].to<JsonObject>();
  f["value"]["doubleValue"] = value;
  f["baseline"]["doubleValue"] = baseline;
}

// Step 2: send one reading
bool sendReading(const char* grade, const float* values, const float* baselines) {
  // Get a new token if we have none or it is about 50 minutes old
  if (idToken == "" || millis() - tokenTime > 50UL * 60UL * 1000UL) {
    if (!signIn()) return false;
  }

  JsonDocument doc;
  JsonObject fields = doc["fields"].to<JsonObject>();
  fields["grade"]["stringValue"] = grade;
  fields["timestamp"]["timestampValue"] = isoTimeNow();
  JsonObject gasFields = fields["gases"]["mapValue"]["fields"].to<JsonObject>();
  for (int i = 0; i < N; i++) addGas(gasFields, KEYS[i], values[i], baselines[i]);
  String payload;
  serializeJson(doc, payload);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = String("https://firestore.googleapis.com/v1/projects/") + PROJECT_ID +
               "/databases/(default)/documents/devices/" + DEVICE_ID + "/readings";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + idToken);
  int code = http.POST(payload);
  if (code != 200) {
    Serial.println(http.getString());
    if (code == 401) idToken = "";   // token rejected: log in again next time
  }
  http.end();
  return code == 200;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");

  configTime(0, 0, "pool.ntp.org");              // get real time from the internet
  while (time(nullptr) < 1700000000) delay(500); // wait until the clock has synced
  Serial.println("Clock synced");
}

void loop() {
  // FAKE data for testing. Replace with the real sensor readings and grade.
  float values[N]    = {1.2, 430, 12, 55, 9};
  float baselines[N] = {1.0, 420, 10, 50, 8};

  Serial.println(sendReading("B", values, baselines) ? "sent" : "failed");
  delay(30000);   // upload every 30 seconds
}