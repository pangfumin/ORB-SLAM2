/*
 * testchar.c
 *
 *  Created on: Nov 15, 2015
 *      Author: sujiwo
 */

#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdio.h>



int userHasPressedKey ()
{
	fd_set fdset;
	struct timeval timeout;
	int  rc;
	int  val;

	timeout.tv_sec = 6;   /* wait for 6 seconds for data */
	timeout.tv_usec = 1000;

	FD_ZERO(&fdset);
	FD_SET(0, &fdset);

	rc = select(1, &fdset, NULL, NULL, &timeout);
	if (rc == -1)  /* select failed */
	{
	   printf("ERROR path\n");
	   val='E';
	}
	else if (rc == 0)  /* select timed out */
	{
		return 0;
	}
	else
	{
		if (FD_ISSET(0, &fdset))
		{
			read (1, &val, 1);
			return 1;
		}
	}
}


int main (int ac, char *av[])
{
	while (1) {
		if (userHasPressedKey() != 0) {
			printf ("Key Pressed\n");
			break;
		}
	}
}
