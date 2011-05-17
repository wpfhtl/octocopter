#ifndef WIRELESSDEVICE_H
#define WIRELESSDEVICE_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>


#include <net/if_arp.h>         /* For ARPHRD_ETHER */
#include <sys/socket.h>         /* For AF_INET & struct sockaddr */
#include <netinet/in.h>         /* For struct sockaddr_in */
#include <netinet/if_ether.h>

/* Fixup to be able to include kernel includes in userspace.
 * Basically, kill the sparse annotations... Jean II */
#ifndef __user
#define __user
#endif

#include <linux/types.h>                /* for "caddr_t" et al          */

/* Glibc systems headers are supposedly less problematic than kernel ones */
#include <sys/socket.h>                 /* for "struct sockaddr" et al  */
//#include <net/if.h>                     /* for IFNAMSIZ and co... */

#include <linux/wireless.h>

#include <QString>
#include <QDebug>



class WirelessDevice
{
private:
    QString mInterfaceName;
    int mSocket;

    int socketOpen(void);
    void socketClose(int skfd);

    int iw_check_mac_addr_type(int skfd, const char* ifname);

    int iw_get_ext(int skfd,           /* Socket to the kernel */
                                 const char *         ifname,         /* Device name */
                                 int                  request,        /* WE ID */
                                 struct iwreq *       pwrq);

public:
    WirelessDevice(const QString& interfaceName);
    ~WirelessDevice();

    qint8 getRssi();
};

#endif // WIRELESSDEVICE_H
