import de.fhpotsdam.unfolding.mapdisplay.*;
import de.fhpotsdam.unfolding.utils.*;
import de.fhpotsdam.unfolding.marker.*;
import de.fhpotsdam.unfolding.tiles.*;
import de.fhpotsdam.unfolding.interactions.*;
import de.fhpotsdam.unfolding.ui.*;
import de.fhpotsdam.unfolding.*;
import de.fhpotsdam.unfolding.core.*;
import de.fhpotsdam.unfolding.mapdisplay.shaders.*;
import de.fhpotsdam.unfolding.data.*;
import de.fhpotsdam.unfolding.geo.*;
import de.fhpotsdam.unfolding.texture.*;
import de.fhpotsdam.unfolding.events.*;
import de.fhpotsdam.utils.*;
import de.fhpotsdam.unfolding.providers.*;

import controlP5.*;
import processing.serial.*;
import org.dishevelled.processing.executor.Executor;
import java.util.concurrent.TimeUnit;

ControlP5 cp5;
Executor executor;
UnfoldingMap map;

DropdownList drp_comports;
DropdownList drp_baudrate;
int baudrates[] = { 115200, 57600, 38400, 28800, 19200, 9600, 4800 };
int i_com = -1;
int i_baud = -1;

Serial port;

Toggle tog_prot;

boolean connected = false;

void setup() {
  size(1000, 600);
  
  executor = new Executor(this, 4);
  
  cp5 = new ControlP5(this);
  
  map = new UnfoldingMap(this);
  MapUtils.createDefaultEventDispatcher(this, map);
  
  // create a DropdownLists
  drp_comports = cp5.addDropdownList("drp_comports")
                    .setPosition(20, 30)
                    ;
  drp_comports.captionLabel().set("Choose port");
  customize(drp_comports); // customize the first list
  for (int i=0;i<Serial.list().length;i++) {
    drp_comports.addItem(Serial.list()[i],i);
  }
  
  drp_baudrate = cp5.addDropdownList("drp_baudrate")
                    .setPosition(121, 30)
                    ;
  drp_baudrate.captionLabel().set("Choose baud");
  customize(drp_baudrate);
  drp_baudrate.addItem("115200",0);
  drp_baudrate.addItem("57600",1);
  drp_baudrate.addItem("38400",2);
  drp_baudrate.addItem("28800",3);
  drp_baudrate.addItem("19200",4);
  drp_baudrate.addItem("9600",5);
  drp_baudrate.addItem("4800",6);
  
  // Protected cutdown toggle
  tog_prot = cp5.addToggle("protected_tog")
                .setPosition(720,10)
                .setSize(50,20)
                .setColorActive(color(100))
                .setMode(ControlP5.SWITCH)
                ;
  tog_prot.getCaptionLabel().align(ControlP5.CENTER, ControlP5.BOTTOM_OUTSIDE).setText("Cutdown Enable");
  
  addMouseWheelListener();
}

void draw() {
  background(128);
  map.draw();
  
  if (!connected) {
     try { 
        port = new Serial(this, Serial.list()[i_com], baudrates[i_baud]);
        port.write("CMD GET SYS VER\r");
        connected = true;
     } catch (Exception e) {
       connected = false;
     }
  } else {
    try {
      if (port.available() > 0) {
        String response = port.readStringUntil('\n');
        if (response != null) {
          print(response);
          //parseResponse(response);
        }
      }
    } catch (Exception e) {
      connected = false;
    }
  }
}

void customize(DropdownList ddl) {
  // a convenience function to customize a DropdownList
  ddl.setBackgroundColor(color(190));
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
  ddl.scroll(0);
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}

void controlEvent(ControlEvent theEvent) {
  // DropdownList is of type ControlGroup.
  // A controlEvent will be triggered from inside the ControlGroup class.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message thrown by controlP5.

  if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    println("event from group : "+theEvent.getGroup().getValue()+" from "+theEvent.getGroup());
    if (theEvent.getGroup().getName() == "drp_comports") {
      i_com = (int)drp_comports.getValue();
      if (port != null) {
        port.clear();
        port.stop();
        connected = false;
      }
    }
    if (theEvent.getGroup().getName() == "drp_baudrate") {
      i_baud = (int)drp_baudrate.getValue();
      if (port != null) {
        port.clear();
        port.stop();
        connected = false;
      }
    }
  }
  else if (theEvent.isController()) {
    println("event from controller : "+theEvent.getController().getValue()+" from "+theEvent.getController());
  }
}

void addMouseWheelListener() {
  frame.addMouseWheelListener(new java.awt.event.MouseWheelListener() {
    public void mouseWheelMoved(java.awt.event.MouseWheelEvent e) {
      cp5.setMouseWheelRotation(e.getWheelRotation());
    }
  }
  );
}
