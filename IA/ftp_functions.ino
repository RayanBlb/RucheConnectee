/*  FTP RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/
#include <LITTLEFS.h>
#include <WiFiClient.h>

void readAndSendBigBinFile(ESP32_FTPClient ftpClient) {
  ftpClient.InitFile("Type I");
  char * path = (char*)malloc(128);
  *path = '\0';
  strcat(path, get_datetime());
  strcat(path, ".wav");

  ftpClient.NewFile(path);

  String fullPath = "/recording.wav";
  Serial.print("Reading file: ");
  Serial.println(fullPath);

  File file = LITTLEFS.open(fullPath);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");

  while (file.available()) {
    // Create and fill a buffer
    unsigned char buf[1024];
    int readVal = file.read(buf, sizeof(buf));
    ftpClient.WriteData(buf, sizeof(buf));
  }
  ftpClient.CloseFile();
  file.close();
}

void upload_to_ftp() {
  ESP32_FTPClient ftp (FTP_SERVER, FTP_USER, FTP_PASSWORD, TIMEOUT, 2);
  ftp.OpenConnection();
  ftp.ChangeWorkDir("/");
  ftp.InitFile("Type A");
  // On va dans le dossier de la board
  ftp.ChangeWorkDir(SERIAL_NUMBER);
  // On upload le fichier
  readAndSendBigBinFile(ftp);
  ftp.CloseConnection();
}
