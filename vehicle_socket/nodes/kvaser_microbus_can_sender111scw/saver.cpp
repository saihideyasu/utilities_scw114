#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

#define BUFFER_SIZE 256

int main() {
  /* ポート番号、ソケット */
  unsigned short port = 9876;
  int srcSocket;  // 自分
  int dstSocket;  // 相手

  /* sockaddr_in 構造体 */
  struct sockaddr_in srcAddr;
  struct sockaddr_in dstAddr;
  socklen_t dstAddrSize = sizeof(dstAddr);

  /* 各種パラメータ */
  int numrcv;
  char buffer[BUFFER_SIZE];

  /************************************************************/
  /* sockaddr_in 構造体のセット */
  memset(&srcAddr, 0, sizeof(srcAddr));
  srcAddr.sin_port = htons(port);
  srcAddr.sin_family = AF_INET;
  srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* ソケットの生成 */
  srcSocket = socket(AF_INET, SOCK_STREAM, 0);

  /* ソケットのバインド */
  bind(srcSocket, (struct sockaddr *) &srcAddr, sizeof(srcAddr));

  /* 接続の許可 */
  listen(srcSocket, 1);

  /* 接続の受付け */
  printf("Waiting for connection ...\n");
  dstSocket = accept(srcSocket, (struct sockaddr *) &dstAddr, &dstAddrSize);
  printf("Connected from %s\n", inet_ntoa(dstAddr.sin_addr));

  /* パケット受信 */
  int i=0;
  while(1) {
    /*numrcv = recv(dstSocket, buffer, BUFFER_SIZE, 0); 
    if(numrcv == 0 || numrcv == -1) {
      close(dstSocket); break;
    }
    printf("received: %s\n", buffer);*/

    printf("sending...\n");
    char buf[7];
    sprintf(buf, "abesi%d", i);
    send(dstSocket, buf, sizeof(buf), 0);
    i++;
    if(i > 10) i=0;
    sleep(1);
  }
}
