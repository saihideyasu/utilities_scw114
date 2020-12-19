#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

int main() {
  /* IP アドレス、ポート番号、ソケット */
  char destination[80];
  unsigned short port = 100;
  int dstSocket;

  /* sockaddr_in 構造体 */
  struct sockaddr_in dstAddr;

  /* 各種パラメータ */
  int status;
  int numsnt;
  char *toSendText = "This is a test";

  /************************************************************/
  /* 相手先アドレスの入力 */
  printf("Connect to ? : (name or IP address) ");
  //scanf("%s", destination);

  /* sockaddr_in 構造体のセット */
  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_port = htons(port);
  dstAddr.sin_family = AF_INET;
  dstAddr.sin_addr.s_addr = inet_addr("192.168.1.10");
 
  /* ソケット生成 */
  dstSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(dstSocket < 0) std::cout << "error" << std::endl;
  /* 接続 */
  printf("Trying to connect to %s: \n", destination);
  int connect_ret = connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr));
  printf("connect %d\n", connect_ret);

  /* パケット送出 */
  for(int i=0; i<1000; i++) {
    /*printf("sending...\n");
    send(dstSocket, toSendText, strlen(toSendText)+1, 0);
    sleep(1);*/

    unsigned char buf[500];
    int read_size=read(dstSocket, &buf, 500);
    std::cout << read_size << std::endl;
    for(int j=0;j<read_size;j++)
    {
      /*unsigned char tmp = 0;
      for(int k=0;k<8;k++)
      {
        unsigned char a = buf[j] & ((unsigned char)0x01 << k);
        a >>= k;
        tmp |= a << (7-k);
      }
      buf[j] = tmp;*/
      printf("%c,", buf[j]);
    }
    printf("\n");


  }

  /* ソケット終了 */
  close(dstSocket);
}