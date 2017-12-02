

import java.awt.Button;
import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.Label;
import java.awt.Panel;
import java.awt.Point;
import java.awt.Robot;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.event.MouseInputListener;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import gnu.io.CommPortIdentifier; 
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent; 
import gnu.io.SerialPortEventListener; 
import java.util.Enumeration;

public class Rajasthan_Processor {
  private JLabel label;
  
  ArrayList<Integer> X = new ArrayList<Integer>();
  ArrayList<Integer> Y = new ArrayList<Integer>();

  private Point clickPoint, cursorPoint;

  static Robot robot;
  
  private void buildUI(Container container) {
    container.setLayout(new BoxLayout(container, BoxLayout.PAGE_AXIS));

    CoordinateArea coordinateArea = new CoordinateArea(this);
    container.add(coordinateArea);

    label = new JLabel();
    resetLabel();
    container.add(label);

    coordinateArea.setAlignmentX(Component.LEFT_ALIGNMENT);
    label.setAlignmentX(Component.LEFT_ALIGNMENT); // redundant
  }

  public void updateCursorLocation(int x, int y) {
    if (x < 0 || y < 0) {
      cursorPoint = null;
      updateLabel();
      return;
    }

    if (cursorPoint == null) {
      cursorPoint = new Point();
    }

    cursorPoint.x = x;
    cursorPoint.y = y;
    robot.mouseMove(x, y);;
    updateLabel();
  }

  public void updateClickPoint(Point p) {
    clickPoint = p;
   // robot.mousePress(InputEvent.BUTTON1_MASK);
    updateLabel();
  }

  public void resetLabel() {
    cursorPoint = null;
    updateLabel();
  }

  protected void updateLabel() {
    String text = "";

    if ((clickPoint == null) && (cursorPoint == null)) {
      text = "Click or move the cursor within the framed area.";
    } else {

      if (clickPoint != null) {
        text += "The last click was at (" + clickPoint.x + ", " + clickPoint.y + "). ";
      }

      if (cursorPoint != null) {
        text += "The cursor is at (" + cursorPoint.x + ", " + cursorPoint.y + "). ";
      }
    }

    label.setText(text);
  }
  static JFrame frame,fr;
  static Label l;
  static Button b = new Button("CALIBRATE");
  void r(){
	  BufferedImage cursorImg = new BufferedImage(16, 16, BufferedImage.TYPE_INT_ARGB);
	    Cursor blankCursor = Toolkit.getDefaultToolkit().createCustomCursor(cursorImg, new Point(0,0), "blank cursor");
	    frame.getContentPane().setCursor(blankCursor);
  }
  public static void main(String[] args) throws Exception {

	robot = new Robot();  
	  
    frame = new JFrame("FRAME CALIBRATION");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setSize(1600,1200);
    
    fr = new JFrame("CALIBRATE OFFSETS");
    fr.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    fr.setSize(600,300);
    fr.setLayout(new GridLayout(2,1));
    b.setBackground(Color.WHITE);
    //fr.pack();
    //fr.setVisible(true);
    l = new Label();
    fr.add(l);
    fr.add(b);
    fr.setVisible(true);
    l.setText("LOADING ...");
    b.setVisible(false);
    frame.setExtendedState(Frame.MAXIMIZED_BOTH);
   /* BufferedImage cursorImg = new BufferedImage(16, 16, BufferedImage.TYPE_INT_ARGB);
    Cursor blankCursor = Toolkit.getDefaultToolkit().createCustomCursor(cursorImg, new Point(0,0), "blank cursor");
    frame.getContentPane().setCursor(blankCursor);*/
    
    Rajasthan_Processor controller = new Rajasthan_Processor();
    controller.buildUI(frame.getContentPane());

    //frame.pack();
    //frame.setVisible(true);
  }

  public static class CoordinateArea extends JComponent implements MouseInputListener, SerialPortEventListener {
    Point point = null;

    Rajasthan_Processor controller;

    Dimension preferredSize = new Dimension(400, 75);

    Color gridColor;
    
    SerialPort serialPort_gest;
    SerialPort serialPort_led;

   
  private static final String PORT_NAMES_GEST[] = { 
  		"/dev/tty.usbserial-A9007UX1", // Mac OS X
                    "/dev/ttyACM0", // Raspberry Pi`
  		"/dev/ttyUSB0", // Linux
  		"COM9", // Windows
  };
  private static final String PORT_NAMES_LED[] = { 
	  		"/dev/tty.usbserial-A9007UX1", // Mac OS X
	                    "/dev/ttyACM0", // Raspberry Pi
	  		"/dev/ttyUSB0", // Linux
	  		"COM5", // Windows
	  };
  private static BufferedReader input;
  private static OutputStream output;
  private static final int TIME_OUT = 2000;
  private static final int DATA_RATE = 74880;

    public CoordinateArea(Rajasthan_Processor controller) {
      this.controller = controller;

      setBorder(BorderFactory.createMatteBorder(1, 5, 5, 1, Color.RED));

      addMouseListener(this);
      addMouseMotionListener(this);
      setBackground(Color.WHITE);
      setOpaque(true);
    ////////GESTURE CONTROLLER  
  	CommPortIdentifier portId_gest = null;
  	Enumeration portEnum_gest = CommPortIdentifier.getPortIdentifiers();

  	while (portEnum_gest.hasMoreElements()) {
  		CommPortIdentifier currPortId = (CommPortIdentifier) portEnum_gest.nextElement();
  		for (String portName : PORT_NAMES_GEST) {
  			if (currPortId.getName().equals(portName)) {
  				portId_gest = currPortId;
  				break;
  			}
  		}
  	}
  	if (portId_gest == null) {
  		System.out.println("Could not find COM port.");
  		l.setText("Could not find COM port."); 
  		return;
  	}

  	try {
  		serialPort_gest = (SerialPort) portId_gest.open(this.getClass().getName(),
  				TIME_OUT);

  		serialPort_gest.setSerialPortParams(DATA_RATE,
  				SerialPort.DATABITS_8,
  				SerialPort.STOPBITS_1,
  				SerialPort.PARITY_NONE);

  		input = new BufferedReader(new InputStreamReader(serialPort_gest.getInputStream()));

  		serialPort_gest.addEventListener((SerialPortEventListener) this);
  		serialPort_gest.notifyOnDataAvailable(true);
  	} catch (Exception e) {
  		System.err.println(e.toString());
  	}
  	Thread t=new Thread() {
  		public void run() {
  			try {Thread.sleep(1000000);} catch (InterruptedException ie) {}
  		}
  	};
  	t.start();
  	System.out.println("Started");
  	l.setText("Serial link Started");
  	
  //////DEVICE CONTROLLER	
  	
  	CommPortIdentifier portId = null;
  	Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

  	while (portEnum.hasMoreElements()) {
  		CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
  		for (String portName : PORT_NAMES_LED) {
  			if (currPortId.getName().equals(portName)) {
  				portId = currPortId;
  				break;
  			}
  		}
  	}
  	if (portId == null) {
  		System.out.println("Could not find COM port.");
  		l.setText("Could not find COM port."); 
  		return;
  	}

  	try {
  		serialPort_led = (SerialPort) portId.open(this.getClass().getName(),
  				TIME_OUT);

  		serialPort_led.setSerialPortParams(DATA_RATE,
  				SerialPort.DATABITS_8,
  				SerialPort.STOPBITS_1,
  				SerialPort.PARITY_NONE);

  		output = serialPort_led.getOutputStream();

  		serialPort_led.addEventListener((SerialPortEventListener) this);
  		serialPort_led.notifyOnDataAvailable(true);
  	} catch (Exception e) {
  		System.err.println(e.toString());
  	}
  	Thread t1=new Thread() {
  		public void run() {
  			try {Thread.sleep(1000000);} catch (InterruptedException ie) {}
  		}
  	};
  	t1.start();
  	System.out.println("Started");
  	l.setText("Serial link Started");
    }

    public Dimension getPreferredSize() {
      return preferredSize;
    }


    protected void paintComponent(Graphics g) {
    	controller.r();
      if (isOpaque()) {
        g.setColor(getBackground());
        g.fillRect(0, 0, getWidth(), getHeight());
      }

      g.setColor(Color.GRAY);
      drawGrid(g, 20);

      if (point != null) {
        g.setColor(getForeground());
        g.fillRect(point.x - 3, point.y - 3, 7, 7);
        for(int i = 0;i<controller.X.size();i++){
        	g.setColor(Color.RED);
            g.fillRect(controller.X.get(i) - 3, controller.Y.get(i) - 3, 7, 7);
        }
      }
      	if(cal_status == 0){
      		g.drawString("MOVEYOUR GYRO TO THIS LOCATION",frame.getWidth()/4-20,frame.getHeight()/2-10);
      		g.fillOval(frame.getWidth()/4,frame.getHeight()/2,10,10);
      		//g.drawOval(frame.getWidth()/4,frame.getHeight()/2,50,50);
      	}
  		else if(cal_status == 1){
  			g.drawString("MOVEYOUR GYRO TO THIS LOCATION",frame.getWidth()/2,frame.getHeight()/4);
  			g.fillOval(frame.getWidth()/2,frame.getHeight()/4,10,10);
  			//g.drawOval(frame.getWidth()/2,frame.getHeight()/4,50,50);

  		}
  		else if(cal_status == 2){
  			g.drawString("MOVEYOUR GYRO TO THIS LOCATION",3*(frame.getWidth()/4),frame.getHeight()/2);
  			g.fillOval(3*(frame.getWidth()/4),frame.getHeight()/2,10,10);
  			//g.drawOval(3*(frame.getWidth()/4),frame.getHeight()/2,50,50);

  		}
  		else if(cal_status == 3){
  			g.drawString("MOVEYOUR GYRO TO THIS LOCATION",frame.getWidth()/2,3*(frame.getHeight()/4));
  			g.fillOval(frame.getWidth()/2,3*(frame.getHeight()/4),10,10);
  			//g.drawOval(frame.getWidth()/2,3*(frame.getHeight()/4),50,50);

  		}
    }

    private void drawGrid(Graphics g, int gridSpace) {
      Insets insets = getInsets();
      int firstX = insets.left;
      int firstY = insets.top;
      int lastX = getWidth() - insets.right;
      int lastY = getHeight() - insets.bottom;

      int x = firstX;
      while (x < lastX) {
        g.drawLine(x, firstY, x, lastY);
        x += gridSpace;
      }

      int y = firstY;
      while (y < lastY) {
        g.drawLine(firstX, y, lastX, y);
        y += gridSpace;
      }
    }

    public void mouseClicked(MouseEvent e) {
     /* int x = e.getX();
      int y = e.getY();

      controller.X.add(x);
      controller.Y.add(y);
      if (point == null) {
        point = new Point(x, y);
      } else {
        point.x = x;
        point.y = y;
      }
      controller.updateClickPoint(point);
      repaint();*/
    }

    public void mouseMoved(MouseEvent e) {
    	/*int x = e.getX();
        int y = e.getY();
        if (point == null) {
          point = new Point(x, y);
        } else {
          point.x = x;
          point.y = y;
        }
        controller.updateCursorLocation(e.getX(), e.getY());
        controller.updateClickPoint(point);
        repaint();*/
      
    }

    public void mouseExited(MouseEvent e) {
      controller.resetLabel();
    }

    public void mouseReleased(MouseEvent e) {
    }

    public void mouseEntered(MouseEvent e) {
    }

    public void mousePressed(MouseEvent e) {
    }

    public void mouseDragged(MouseEvent e) {
    }
  

public synchronized void close() {
	if (serialPort_gest != null) {
		serialPort_gest.removeEventListener();
		serialPort_gest.close();
	}
	if (serialPort_led != null) {
		serialPort_led.removeEventListener();
		serialPort_led.close();
	}
}


static int cal_status = 0;
static double X1,X2,Y1,Y2;
static double ledX, ledY, thres = 35;
static double ledX2, ledY2;
static boolean clk_sts = false;
static boolean prev_sts = false;
static boolean led_sts = false;
static boolean led_sts2 = false;
public synchronized void serialEvent(SerialPortEvent oEvent) {
	if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
		try {
			String inputLine=input.readLine();
			System.out.println(inputLine);
			String temp="";
			for(int i = 0;i<inputLine.length() && inputLine.charAt(i) != ' ';++i){
				temp+=inputLine.charAt(i);
			}
			fr.setLocationRelativeTo(null);
			if(!temp.equals("ypr")){
				//l.setText(inputLine);
				fr.setVisible(true);
				if(temp.equals("Calibrate")){
					b.setVisible(true);
					l.setText(inputLine+" : PLEASE KEEP YOUR GYRO STEADILY");
					b.addActionListener(new ActionListener() {
			            @Override
			            public void actionPerformed(ActionEvent e) {
			                try {
								output.write('e');
								b.setEnabled(false);
							} catch (IOException e1) {
								e1.printStackTrace();
							}
			            }
			        });
					
				}
				else if(temp.equals("...")){
					l.setText("Calibrating .. PLEASE WAIT AND DON't MOVE YOUR GYRO "); 
					b.setVisible(false);
				}
				/*else if(temp.equals("con")){
					fr.setVisible(true);
					l.setText("Gyro is switched off");
					if(cal_status<=4){
						frame.setVisible(false);
					}
				}*/
				else{
					l.setText(inputLine); 
				}

			}
			if (temp.equals("ypr")) {
				//frame.setVisible(true);
				fr.setVisible(false);
				
				temp="";
				double[] xyz = new double[3];
				int L = 0,T=0;
				int count = -1;
				for (int i = 0; i < inputLine.length(); ++i) {
					if (inputLine.charAt(i) == ' ') {
						if (count == -1) {
							temp = "";
							count++;
							continue;
						}
						if(count == 3){
							L = Integer.parseInt(temp);
							if(L == 0)
								prev_sts = true;
							count++;
							continue;
						}
						if(count == 4){
							T = Integer.parseInt(temp);
							continue;
						}
						xyz[count++] = Double.parseDouble(temp);
						temp = "";
					} else {
						temp += inputLine.charAt(i);
					}
				}
				if(prev_sts && L == 1){
					clk_sts = true;
					prev_sts = false;
				}
				if(L==0){
					//robot.mousePress(InputEvent.BUTTON1_DOWN_MASK);
				}
				System.out.println(xyz[0] + " " + xyz[1] + " " + xyz[2] + " " + clk_sts + " " + T);
				if(cal_status<=5){
					frame.setVisible(true);
					if(clk_sts){
						if(cal_status == 0){
				    		CoordinateArea.X1 = xyz[0];
				    	}
				    	else if(cal_status == 1){
				    		CoordinateArea.Y1 = xyz[1];
				    	}
				    	else if(cal_status == 2){
				    		CoordinateArea.X2 = xyz[0];
				    	}
				    	else if(cal_status == 3){
				    		CoordinateArea.Y2 = xyz[1];
				    		frame.setVisible(false);
				    	}
				    	else if(cal_status == 4){
				    		CoordinateArea.ledX = xyz[0];
				    		CoordinateArea.ledY = xyz[1];
				    		frame.setVisible(false);
				    		
				    	}
						else {
							CoordinateArea.ledX2 = xyz[0];
				    		CoordinateArea.ledY2 = xyz[1];
				    		frame.setVisible(false);
				    	}
				    	cal_status++;
				    	repaint();
					}
				}
				else {
					double x = xyz[0], y = xyz[1],
							xMin = CoordinateArea.X1 - (CoordinateArea.X2 - CoordinateArea.X1)/2,
							xMax = CoordinateArea.X2 + (CoordinateArea.X2 - CoordinateArea.X1)/2, 
							yMin = CoordinateArea.Y1 - (CoordinateArea.Y2 - CoordinateArea.Y1)/2, 
							yMax = CoordinateArea.Y2 + (CoordinateArea.Y2 - CoordinateArea.Y1)/2,
							X = frame.getWidth(), Y = frame.getHeight();
					int x1 = (int) (((x - xMin) / (xMax - xMin)) * X);
					int y1 = (int) (((y - yMin) / (yMax - yMin)) * Y);
					
					//if (T == 0) {
						if (point == null) {
							point = new Point(x1, y1);
						} else {
							point.x = x1;
							point.y = y1;
						}
						if (clk_sts) {
							controller.X.add(x1);
							controller.Y.add(y1);
							robot.mousePress(InputEvent.BUTTON1_DOWN_MASK);
							robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);
							
							if(x<(CoordinateArea.ledX+CoordinateArea.thres) && x>(CoordinateArea.ledX-CoordinateArea.thres) && y<(CoordinateArea.ledY+CoordinateArea.thres) && y>(CoordinateArea.ledY-CoordinateArea.thres)){
								if(CoordinateArea.led_sts){
									CoordinateArea.led_sts = false;
				        			try {
				        				output.write('1');
				        			} catch (IOException e1) {
				        				e1.printStackTrace();
				        			}
				                }
				                else{
				                	CoordinateArea.led_sts = true;
				        			try {
				        				output.write('0');
				        			} catch (IOException e1) {
				        				e1.printStackTrace();
				        			};
				                }
							}
							
							if(x<(CoordinateArea.ledX2+CoordinateArea.thres) && x>(CoordinateArea.ledX2-CoordinateArea.thres) && y<(CoordinateArea.ledY2+CoordinateArea.thres) && y>(CoordinateArea.ledY2-CoordinateArea.thres)){
								if(CoordinateArea.led_sts2){
									CoordinateArea.led_sts2 = false;
				        			try {
				        				output.write('3');
				        			} catch (IOException e1) {
				        				e1.printStackTrace();
				        			}
				                }
				                else{
				                	CoordinateArea.led_sts2 = true;
				        			try {
				        				output.write('2');
				        			} catch (IOException e1) {
				        				e1.printStackTrace();
				        			};
				                }
							}
						}
						controller.updateCursorLocation(x1, y1);
						controller.updateClickPoint(point);
						repaint();
					//}		
						
				}
				clk_sts = false;
			}	
		} catch (Exception e) {
			System.err.println(e.toString());
		}
	}
}
}
}
