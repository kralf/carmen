#include <qapplication.h>

#include "global.h"

#include "proccontrol_interface.h"
#include "proccontrol_gui.h"

QDisplay                             * qdisplay;
int                                    pid_update = FALSE;
int                                    out_update = FALSE;

carmen_proccontrol_pidtable_message     pidtable;
carmen_proccontrol_output_message       output;

int
output_pid( int pid )
{
  int i, j;
  for (i=0; i<qdisplay->table.numgrps; i++) {
    for (j=0; j<qdisplay->table.procingrp[i]; j++) {
      if (qdisplay->table.process[i][j].pid==pid) {
  return(qdisplay->table.process[i][j].output);
      }
    }
  }
  return(FALSE);
}

void
carmen_update_pidtable( carmen_proccontrol_pidtable_message *msg )
{
  static process_table_type  p;
  int                        i, j, g, m, newgrp;

  for (i=0; i<p.numgrps; i++) {
    p.procingrp[i] = 0;
  }
  p.numgrps = 0;

  for (i=0; i<msg->num_processes; i++) {
    newgrp = TRUE;
    for (j=0; j<p.numgrps; j++) {
      if (!strncmp(msg->process[i].group_name,p.process[j][0].group_name,
       MAX_NAME_LENGTH)) {
  g = j;
  newgrp = FALSE;
  break;
      }
    }
    if (newgrp) {
      g = p.numgrps++;
    }
    m = p.procingrp[g];
    p.process[g][m].group = g;
    strncpy( p.process[g][m].group_name, msg->process[i].group_name,
       MAX_NAME_LENGTH );
    p.process[g][m].module = m;
    strncpy( p.process[g][m].module_name, msg->process[i].module_name,
       MAX_NAME_LENGTH );
    p.process[g][m].pid = msg->process[i].pid;
    p.process[g][m].active = msg->process[i].active;
    p.process[g][m].requested_state = msg->process[i].requested_state;
    p.procingrp[g]++;
  }
  for (i=0; i<qdisplay->table.numgrps; i++) {
    if (i>=p.numgrps || p.procingrp[i] != qdisplay->table.procingrp[i]) {
      for (j=0; j<qdisplay->table.procingrp[i]; j++) {
  qdisplay->hideButton( i, j );
  qdisplay->table.process[i][j].active = -1;
      }
    }
  }
  for (i=0; i<p.numgrps; i++) {
    if ( p.procingrp[i] != qdisplay->table.procingrp[i] ) {
      qdisplay->setGroup( i, p.process[i][0].group_name );
      for (j=0; j<p.procingrp[i]; j++) {
  qdisplay->table.process[i][j] = p.process[i][j];
  qdisplay->setModule( i, j+1,
           qdisplay->table.process[i][j].module_name,
           qdisplay->table.process[i][j].pid  );
  qdisplay->showStatus( i, j+1, qdisplay->table.process[i][j].active );
      }
      qdisplay->table.procingrp[i] = p.procingrp[i];
    } else {
      for (j=0; j<p.procingrp[i]; j++) {
  if ( strcmp(p.process[i][j].group_name,
        qdisplay->table.process[i][j].group_name) ||
       strcmp(p.process[i][j].module_name,
        qdisplay->table.process[i][j].module_name) ||
       p.process[i][j].pid != qdisplay->table.process[i][j].pid ||
       p.process[i][j].active != qdisplay->table.process[i][j].active ||
       p.process[i][j].requested_state !=
       qdisplay->table.process[i][j].requested_state ) {
    p.process[i][j].output = qdisplay->table.process[i][j].output;
    qdisplay->table.process[i][j] = p.process[i][j];
    qdisplay->setModule( i, j+1,
             p.process[i][j].module_name,
             p.process[i][j].pid  );
    qdisplay->showStatus( i, j+1, p.process[i][j].active );
  }
      }
    }
  }
  qdisplay->table.numgrps = p.numgrps;

}

void
carmen_proccontrol_pidtable_handler( carmen_proccontrol_pidtable_message *msg __attribute__ ((unused)) )
{
  pid_update = TRUE;
}

void
carmen_output_handler( carmen_proccontrol_output_message *msg __attribute__ ((unused)) )
{
  out_update = TRUE;
}

void
carmen_output( int state )
{
  if (state) {
    fprintf( stderr, "INFO: subscribe output messages\n" );
    carmen_proccontrol_subscribe_output_message
      ( &output,
  (carmen_handler_t) carmen_output_handler,
  CARMEN_SUBSCRIBE_ALL );
  } else {
    fprintf( stderr, "INFO: unsubscribe output\n" );
    carmen_proccontrol_unsubscribe_output_message
      ( (carmen_handler_t) carmen_output_handler );
  }
}

void
shutdown( int sig ) {
  exit(sig);
}

int
main( int argc, char** argv)
{
  QApplication         app( argc, argv );
  QDisplay             gui;


  qdisplay = &gui;

  carmen_ipc_initialize(argc, argv);

  carmen_proccontrol_subscribe_pidtable_message
    ( &pidtable,
      (carmen_handler_t ) carmen_proccontrol_pidtable_handler,
      CARMEN_SUBSCRIBE_ALL );

  signal(SIGINT,shutdown);
  gui.show();

  while (TRUE) {
    app.processEvents();
    if (pid_update) {
      carmen_update_pidtable( &pidtable );
      pid_update = FALSE;
    }
    if (out_update) {
      if (output_pid( output.pid ))
	qdisplay->showLine( output.output );
      out_update = FALSE;
    }
    carmen_ipc_sleep (0.02);
  }
}
