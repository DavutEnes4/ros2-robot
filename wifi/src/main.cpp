#include <Arduino.h>
#include <painlessMesh.h>
#include <ArduinoJson.h>
#include <deque>

#define   MESH_PREFIX "mesh"
#define   MESH_PASSWORD   "mesh12345678"
#define   MESH_PORT   5555

Scheduler userScheduler;
painlessMesh  mesh;

String inputBuffer = "";
bool commandComplete = false;
uint32_t id;

#define MAX_ID_LSIT_SIZE 10
#define MESSAGE_MS 60000

struct MesajKaydi
{	
	String msg_id;
	unsigned long timestamp;
};

std::deque<MesajKaydi> alinanMesajlar;

bool isNewMessage(String msg_id){
	unsigned long now = millis();

	while (!alinanMesajlar.empty() && (now - alinanMesajlar.front().timestamp > MESSAGE_MS))
	{
		alinanMesajlar.pop_front();
	}

	for (const auto& kayit: alinanMesajlar){
		if (kayit.msg_id == msg_id){
			return false;
		}
	}

	alinanMesajlar.push_back({msg_id,now});
	if (alinanMesajlar.size() > MAX_ID_LSIT_SIZE)
	{
		alinanMesajlar.pop_front();
	}

	return true;
}

void receivedCallback(uint32_t from, String &msg) {
	StaticJsonDocument doc(512);
	DeserializationError error = deserializeJson(doc,msg);

	if(error) {
		Serial.printf("JSON çzöümlemesinde hata oluştu: %s\n",msg.c_str());
		return;
	}

	if(!doc.containsKey("msg_id") || !doc.containsKey("alici_id") || !doc.containsKey("mesaj"))
	{
		Serial.println("Eksik JOSN belgeleri");
		return;
	}	
	
	String msg_id = doc["msg_id"];

	if(!isNewMessage(msg_id))
		return;

	uint32_t hedef = doc["alici_id"];
	String icerik = doc["mesaj"];

	if (hedef == id)
		Serial.printf("[%u] size gelen mesaj: %s \n",from,icerik.c_str());
	else
		mesh.sendBroadcast(inputBuffer);
}

void setup() {
	Serial.begin(115200);
	// # mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);
	id = mesh.getNodeId();
	Serial.printf("Benim Node ID: %u\n", id);

}

String generateMessageId() {
	return String(id) + "_" + String(millis());
}

void executeCommand(String command)
{
	command.trim();

	if (command.startsWith("LIST"))
	{
		auto nodelar = mesh.getNodeList();
		for (auto id : nodelar)
			Serial.printf("Node: %u\n", id);
	}
	else if (command == "ME")
		Serial.printf("My ID:%d\n",id);
	else if (command.startsWith("SEND")) {
		int ilkBosluk = command.indexOf(' ');
		int ikinciBosluk = command.indexOf(' ', ilkBosluk + 1);

		if (ilkBosluk != -1 && ikinciBosluk != -1) {
			String idStr = command.substring(ilkBosluk + 1, ikinciBosluk);
			String mesaj = command.substring(ikinciBosluk + 1);

			uint32_t hedefID = idStr.toInt();

			StaticJsonDocument doc(512);
			doc["msg_id"] = generateMessageId();
			doc["gonderen_id"] = id;
			doc["alici_id"] = hedefID;
			doc["mesaj"] = mesaj;

			String cikti;
			serializeJson(doc,cikti);
			mesh.sendBroadcast(cikti);
			Serial.printf(" Node %u'ya mesaj gönderildi: %s\n", hedefID, mesaj.c_str());
		}
	}
	else
	{
		Serial.println("Bilinmeyen komut. Komutlar: SEND <id> <mesaj>, LIST, ME");
	}
}

void fSendProcess(){
	inputBuffer = "";
	// Seri porttan komutları oku
	while (Serial.available() > 0 && !commandComplete)
	{
		char inChar = (char)Serial.read();
		if (inChar == '\n')
		{
			commandComplete = true;
		}
		else
		{
			inputBuffer += inChar;
		}
	}

	// Veri okuma tamamlandıysa gönder ve sil
	if (commandComplete)
	{
		executeCommand(inputBuffer);
		inputBuffer = "";
		commandComplete = false;
	}
};

void loop() {
	fSendProcess();
	mesh.update();
}
