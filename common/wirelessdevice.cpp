#include "wirelessdevice.h"

WirelessDevice::WirelessDevice(const QString& interfaceName)
{
    mInterfaceName = interfaceName;
    mSocket = socketOpen();
}

WirelessDevice::~WirelessDevice()
{
        socketClose(mSocket);
}

/*
 * Open a socket.
 * Depending on the protocol present, open the right socket. The socket
 * will allow us to talk to the driver.
 */
int WirelessDevice::socketOpen(void)
{
  static const int families[] = {
    AF_INET, AF_IPX, AF_AX25, AF_APPLETALK
  };
  unsigned int  i;
  int           sock;

  /*
   * Now pick any (exisiting) useful socket family for generic queries
   * Note : don't open all the socket, only returns when one matches,
   * all protocols might not be valid.
   * Workaround by Jim Kaba <jkaba@sarnoff.com>
   * Note : in 99% of the case, we will just open the inet_sock.
   * The remaining 1% case are not fully correct...
   */

  /* Try all families we support */
  for(i = 0; i < sizeof(families)/sizeof(int); ++i)
    {
      /* Try to open the socket, if success returns it */
      sock = socket(families[i], SOCK_DGRAM, 0);
      if(sock >= 0)
        return sock;
  }

  qWarning() << "WirelessDevice::socketOpen(): couldn't open socket.";
  return -1;
}

void WirelessDevice::socketClose(int skfd)
{
  close(skfd);
}

int WirelessDevice::iw_get_ext(
    int skfd,           /* Socket to the kernel */
           const char *         ifname,         /* Device name */
           int                  request,        /* WE ID */
           struct iwreq *       pwrq)           /* Fixed part of the request */
{
  /* Set device name */
  strncpy(pwrq->ifr_name, ifname, IFNAMSIZ);
  /* Do the request */
  return(ioctl(skfd, request, pwrq));
}

int WirelessDevice::iw_check_mac_addr_type(int skfd, const char* ifname)
{
  struct ifreq          ifr;

  /* Get the type of hardware address */
  strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
  if((ioctl(skfd, SIOCGIFHWADDR, &ifr) < 0) ||
     ((ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER)
      && (ifr.ifr_hwaddr.sa_family != ARPHRD_IEEE80211)))
    {
      /* Deep trouble... */
      fprintf(stderr, "Interface %s doesn't support MAC addresses\n",
             ifname);
      return(-1);
    }

  return(0);
}

qint8 WirelessDevice::getRssi()
{
    char* ifname = mInterfaceName.toAscii().data();

//    print_spy_info(mSocket, argv[1], NULL, 0);

    struct iwreq          wrq;
    char          buffer[(sizeof(struct iw_quality) +
                          sizeof(struct sockaddr)) * IW_MAX_SPY];
    char          temp[128];
    struct sockaddr *     hwa;
    struct iw_quality *   qual;
//    iwrange       range;
    int           has_range = 0;
    int           n;
    int           i;

    /* Avoid "Unused parameter" warning */
//    args = args; count = count;

    /* Collect stats */
    wrq.u.data.pointer = (caddr_t) buffer;
    wrq.u.data.length = IW_MAX_SPY;
    wrq.u.data.flags = 0;
    if(iw_get_ext(mSocket, ifname, SIOCGIWSPY, &wrq) < 0)
      {
        fprintf(stderr, "%-8.16s  Interface doesn't support wireless statistic collection\n\n", ifname);
        return(-1);
      }

    /* Number of addresses */
    n = wrq.u.data.length;

    /* Check if we have valid mac address type */
    if(iw_check_mac_addr_type(mSocket, ifname) < 0)
      {
        fprintf(stderr, "%-8.16s  Interface doesn't support MAC addresses\n\n", ifname);
        return(-2);
      }

    /* Get range info if we can */
//    if(iw_get_range_info(mSocket, ifname, &(range)) >= 0)
//      has_range = 1;

    /* Display it */
    if(n == 0)
      printf("%-8.16s  No statistics to collect\n", ifname);
    else
      printf("%-8.16s  Statistics collected:\n", ifname);

    /* The two lists */
    hwa = (struct sockaddr *) buffer;
    qual = (struct iw_quality *) (buffer + (sizeof(struct sockaddr) * n));

    qDebug() << "quality:" << qual->qual << qual->level << qual->noise << qual->updated;

    return qual->qual;
}
