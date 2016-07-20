/*------------------------------------------------------------------------
 *---------------------           ARGON               --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/10
 *
 *
 *  File: argon.h
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#ifndef ARGON_H_
#define ARGON_H_

#define ARGO_MAX_SWITCH 10

int argo_addInt(int * var, char* sw, int dfl, int need_value);
int argo_addDouble(double * var, char* sw, double dfl);
int argo_addString(char * var, char* sw, char * dfl);
void argo_setComment(char * sw, char * text);
void argo_setCommentId(int id, char * text);
void argo_doProcess(int argc, char * argv[], int start);

/* CODE >>>>>>> */

#define ARGON_INT 0
#define ARGON_STRING  1
#define ARGON_DOUBLE  2
#define WMP_ERROR fprintf
#define STR strdup

typedef struct {
	char sw[16];
	char type;
	void * var;
	int need_value;
	int mandatory;
	int set;
	char comment[64];
} argo_data_t;

static struct {
	argo_data_t at[ARGO_MAX_SWITCH];
	char idx;
} regs;

static char example[256];

static void init(){
	static int initied = 0;
	if (!initied){
		memset(&regs,0,sizeof(regs));
		initied = 1;
		example[0] = 0;
	}

}

int argo_addInt(int * var, char* sw, int dfl, int need_value) {
	init();
	*var = dfl;
	regs.at[(int)regs.idx].var = (void*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_INT;
	regs.at[(int)regs.idx].need_value = need_value;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type int", sw);
	regs.idx++;
	return regs.idx - 1;
}
int argo_addIntMandatory(int * var, char* sw, int dfl, int need_value) {
	init();
	*var = dfl;
	regs.at[(int)regs.idx].var = (void*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_INT;
	regs.at[(int)regs.idx].need_value = need_value;
	regs.at[(int)regs.idx].mandatory = 1;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type int", sw);
	regs.idx++;
	return regs.idx - 1;
}
int argo_addDouble(double * var, char* sw, double dfl) {
	init();
	*var = dfl;
	regs.at[(int)regs.idx].var = (double*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_DOUBLE;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type double", sw);
	regs.idx++;
	return regs.idx - 1;
}

int argo_addString(char * var, char* sw, char * dfl) {
	init();
	strcpy(var, dfl);
	regs.at[(int)regs.idx].var = (char*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_STRING;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type string", sw);
	regs.idx++;
	return regs.idx - 1;
}

void argo_setComment(char * sw, char * text) {
	int i;
	for (i = 0; i < regs.idx; i++) {
		if (strcmp(sw, regs.at[i].sw) == 0) {
			sprintf(regs.at[i].comment, "-%s: %s", sw, text);
		}
	}
}

void argo_setCommentId(int id, char * text){
	sprintf(regs.at[id].comment, "--%s: %s", regs.at[id].sw, text);
}
void argo_setExample(char * exe, char * switchs){
	sprintf(example, "Example:\n %s %s",exe, switchs);
}
void argo_doProcess(int argc, char * argv[], int start) {
	int i, j;
   int done;
	int error = 0;
	for (i = start + 1; i < argc; i++) {

		/* switch format */
		if (argv[i][0] != '-' || (argv[i][1] != '-')) {
			continue;
		}

		/* switch existence */
		done = 0;
		for (j = 0; j < regs.idx; j++) {
			if (strcmp(&argv[i][2], "h") == 0) {
				error = 1;
				done = 1;
				break;
			}

			if (strcmp(&argv[i][2], regs.at[j].sw) == 0) {
				done = 1;
				if (regs.at[j].type == ARGON_INT) {
					if (regs.at[j].need_value) {
						if (i == argc - 1) {
							error = 4;
							break;
						} else {
							*((int *) regs.at[j].var) = atoi(argv[i + 1]);
							regs.at[j].set = 1;
							i++;
						}
					} else {
						*((int *) regs.at[j].var) = 1;
						regs.at[j].set = 1;
					}
				} else if (regs.at[j].type == ARGON_DOUBLE) {
					if (i == argc - 1) {
						error = 4;
						break;
					} else {
						(*((double *) regs.at[j].var))=atof(argv[i + 1]);
						regs.at[j].set = 1;
						i++;
					}
				} else if (regs.at[j].type == ARGON_STRING) {
					if (i == argc - 1) {
						error = 4;
						break;
					} else {
						strcpy(((char *) regs.at[j].var), argv[i + 1]);
						regs.at[j].set = 1;
						i++;
					}
				}
			}
		}

		if (!done) {
         WMP_ERROR(stderr, "*** Inexistent switch %s specified\n",argv[i]);
			error = 3;
			break;
		}

	}
	for (i = 0; i< regs.idx ; i++){
		if (regs.at[i].mandatory &&  !regs.at[i].set){
			WMP_ERROR(stderr, "*** Parameter %s is mandatory\n", regs.at[i].sw);
			error = 8;
		}
	}

	if (error == 4) {
      WMP_ERROR(stderr, "*** Incorrect number of parameters\n");
	}
	if (error != 0) {
		WMP_ERROR(stderr, "List of switches:\n");
		for (i = 0; i < regs.idx; i++) {
			WMP_ERROR(stderr, "%s\n", regs.at[i].comment);
		}
		if (strlen(example) > 0){
			fprintf(stderr,"%s\n",example);
		}
		exit(1);
	}
}
#endif /* ARGON_H_ */
