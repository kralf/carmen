#ifdef __cplusplus
extern "C" {
#endif

#include "global.h"

#include "proccontrol_interface.h"

#ifdef __cplusplus
}
#endif

#include "proccontrol_gui.h"

QDisplay::QDisplay( QWidget *parent, const char *name )
 : QWidget( parent, name )
{
  int i, j;

  setCaption( "PROCCONTROL GUI" );

  QVBoxLayout  *vbox = new QVBoxLayout( this );

  for (j=0; j<MAX_NUM_GROUPS; j++) {
    box[j] = new QHBoxLayout( vbox );
    QString s;
    s.sprintf( "Group (%d)", j );
    bgrp[j] = new QButtonGroup( 1,  QGroupBox::Vertical, s, this );
    box[j]->addWidget( bgrp[j] );
    {
      but[j][0] = new QPushButton( bgrp[j] );
      QString s;
      but[j][0]->setText( "All" );
      but[j][0]->setMinimumWidth( 60 );
      but[j][0]->setMaximumWidth( 60 );
      {
	QPopupMenu *menu = new QPopupMenu(but[j][0]);
	menu->insertItem("Start", this, SLOT( startClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES))+0 );
	menu->insertItem("Stop", this, SLOT( stopClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES))+1 );
	but[j][0]->setPopup(menu);
      }
    }
    for (i=1; i<MAX_NUM_MODULES; i++) {
      but[j][i] = new QPushButton( bgrp[j] );
      QString s;
      s.sprintf( "Button\n(%d)", i );
      but[j][i]->setText( s );
      but[j][i]->setMaximumWidth( 100 );
      but[j][i]->setToggleButton( TRUE );
      {
	QPopupMenu *menu = new QPopupMenu(but[j][i]);
	menu->insertItem("Start Program", this, SLOT( startClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+0 );
	menu->insertItem("Stop Program", this, SLOT( stopClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+1 );
	menu->insertItem("Show Output", this, SLOT( showClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+2 );
	menu->insertItem("No Output", this, SLOT( noClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+3 );
	but[j][i]->setPopup(menu);
      }
      but[j][i]->hide();
    }
    bgrp[j]->hide();
  }

  output =  new QTextView( this );
  output->setMaxLogLines(500);
  QFont font( "Courier" );
  font.setPointSize( 8 );
  output->setFont(font);
  vbox->addWidget( output );

  resize( 600, 400 );
}

void
QDisplay::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

void
QDisplay::showLine( char * text )
{
  output->append( text );
}

void
QDisplay::startClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  if (m==0) {
    fprintf( stderr, "INFO: start group %s\n",
	     table.process[g][m].group_name );
    carmen_proccontrol_set_group_state( table.process[g][m].group_name, 1 );
  } else {
    fprintf( stderr, "INFO: start module %s\n",
	     table.process[g][m-1].module_name );
    carmen_proccontrol_set_module_state( table.process[g][m-1].module_name, 1 );
  }
}

void
QDisplay::stopClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  if (m==0) {
    fprintf( stderr, "INFO: stop group %s\n",
	     table.process[g][m].group_name );
    carmen_proccontrol_set_group_state( table.process[g][m].group_name, 0 );
  } else {
    fprintf( stderr, "INFO: stop module %s\n",
	     table.process[g][m-1].module_name );
    carmen_proccontrol_set_module_state( table.process[g][m-1].module_name, 0 );
  }
}

void
QDisplay::showClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  table.process[g][m-1].output = TRUE;
  but[g][m]->setOn(1);
  if (!table.output) {
    carmen_output(TRUE);
    table.output = TRUE;
  }
}

void
QDisplay::noClicked( int n )
{
  int i, j;
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  table.process[g][m-1].output = FALSE;
  but[g][m]->setOn(0);
  if (table.output) {
    table.output = 0;
    for (i=0; i<table.numgrps; i++) {
      for (j=0; j<table.procingrp[i]; j++) {
	table.output += table.process[i][j].output;
      }
    }
    if (table.output)
      table.output = TRUE;
    else
      carmen_output(FALSE);
  }
}

void
QDisplay::showStatus( int group, int module, int status )
{
  int i;
  if (group>=0 && group<MAX_NUM_GROUPS &&
      module>=0 && module<MAX_NUM_MODULES) {
    if (module==0) {
      for (i=0; i<MAX_NUM_MODULES; i++) {
	QPalette pal = but[group][i]->palette();
	if (status) {
	  pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
	} else {
	  pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
	}
	but[group][i]->setPalette(pal);
      }
    } else {
      QPalette pal = but[group][module]->palette();
      if (status) {
	pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
      } else {
	pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
      }
      but[group][module]->setPalette(pal);
    }
  }
}

void
QDisplay::setGroup( int group, char *group_name )
{
  if (group>=0 && group<MAX_NUM_GROUPS) {
    QString s( group_name );
    bgrp[group]->setTitle( s );
    bgrp[group]->show();
  }
}

void
QDisplay::setModule( int group, int module, char * module_name, int pid )
{
  if (group>=0 && group<MAX_NUM_GROUPS &&
      module>=1 && module<MAX_NUM_MODULES) {
    QString s;
    s.sprintf( "%s\npid: %d", module_name, pid );
    but[group][module]->setText( s );
    but[group][module]->show();
  }
}

void
QDisplay::hideButton( int group, int module )
{
  if (module==0) {
    bgrp[group]->hide();
  } else {
    but[group][module]->setOn(0);
    but[group][module]->hide();
  }
}
