using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;

public class CSharpForGIT : MonoBehaviour
{
    Thread mThread;
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;
    IPAddress localAdd;
    TcpListener listener;
    TcpClient client;
    Vector3 receivedPos = Vector3.zero;

    public GameObject car;
    public GameObject wheelFL, wheelFR, wheelRL, wheelRR;
    public RCC_CarControllerV3 carInput;
    public float wheelAng;
    public float throttlePos;
    public float brakeOri, brakePos;
    public float engineTorque;
    public float brakeTorque;
    public float engineRpm;
    public string simTime;
    public string absTime;
    public int mph;
    public float vehPosX, vehPosY, vehPosZ;
    public float vehAngle;
    public float vehAccLon;
    public float vehAccLonOld = 0.0f;
    public float vehAccX, vehAccY, vehAccZ;
    public float vehAccXOld = 0.0f, vehAccYOld = 0.0f, vehAccZOld = 0.0f;

    //set vehicle position
    public float setVehPosX = 0.0f, setVehPosY = 0.0f, setVehPosZ = 0.0f;
    public bool setPos = false;
    public Vector3 vehPosition = Vector3.zero;

    //trigger area array
    public int FLWheelX, FLWheelZ, FRWheelX, FRWheelZ, RLWheelX, RLWheelZ, RRWheelX, RRWheelZ;

    // message used when python sending data to c#, c# needs to take a break before return the message, to prevent the unecessary pause
    public string ctpMessage = "data recieved";

    //debug var
    // public bool test = false;

    //boolean trigger for constantly recieving data
    bool running;

    private void Update()
    {
        //transform.position = receivedPos; //assigning receivedPos in SendAndReceiveData()
        
        //Universal no need to adjust
        // second = (Time.time % 60);
        // minute = Math.Floor(Time.time / 60) % 60;
        // hour   = Math.Floor(Time.time / 3600);
        simTime = Math.Floor(Time.time / 3600).ToString() + ":" + (Math.Floor(Time.time / 60) % 60).ToString() + ":" + (Time.time % 60).ToString("f5");
        absTime = DateTime.Now.Hour.ToString() + ":" + DateTime.Now.Minute.ToString() + ":" + DateTime.Now.Second.ToString() + "." + DateTime.Now.Millisecond.ToString();

        wheelAng = carInput.steerInput;
        throttlePos = (carInput.inputs.throttleInput + 1) / 2;
        brakePos = (carInput.inputs.brakeInput + 1) / 2;
        engineTorque = carInput.engineTorqueHolder;
        brakeTorque = carInput.brakeTorqueHolder;
        engineRpm = carInput.engineRPM;
        mph = carInput.direction == 1 ? (int)(carInput.speed) : -(int)(carInput.speed);

        vehAccLon = (carInput.rigid.velocity.magnitude - vehAccLonOld) / Time.fixedDeltaTime;
        vehAccLonOld = carInput.rigid.velocity.magnitude;

        vehPosX = transform.position.x;
        vehPosY = transform.position.y;
        vehPosZ = transform.position.z;

        vehAngle = transform.eulerAngles.y;

        vehAccX = (carInput.rigid.velocity.x - vehAccXOld) / Time.fixedDeltaTime;
        vehAccXOld = carInput.rigid.velocity.x;
        vehAccY = (carInput.rigid.velocity.y - vehAccYOld) / Time.fixedDeltaTime;
        vehAccYOld = carInput.rigid.velocity.y;
        vehAccZ = (carInput.rigid.velocity.z - vehAccZOld) / Time.fixedDeltaTime;
        vehAccZOld = carInput.rigid.velocity.z;

        // set vehicle position based on recieved value
        vehPosition = new Vector3(setVehPosX, setVehPosY, setVehPosZ);

        // check if true then position overwrite once
        if(setPos == true)
        {
            // turn off the condition once inside to loop to prevent constant overwrite position
            setPos = false;
            transform.position = vehPosition;
        }

        // read the wheel x, z position 
        FLWheelX = (int)wheelFL.transform.position.x;
        FLWheelZ = (int)wheelFL.transform.position.z;
        FRWheelX = (int)wheelFR.transform.position.x;
        FRWheelZ = (int)wheelFR.transform.position.z;
        RLWheelX = (int)wheelRL.transform.position.x;
        RLWheelZ = (int)wheelRL.transform.position.z;
        RRWheelX = (int)wheelRR.transform.position.x;
        RRWheelZ = (int)wheelRR.transform.position.z;


    }

    private void Start()// server start DON'T TOUCH
    {
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
    }

    void GetInfo() // server get DON'T TOUCH
    {
        localAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();

        client = listener.AcceptTcpClient();

        running = true;
        while (running)
        {
            SendAndReceiveData();
        }
        listener.Stop();
    }

    void SendAndReceiveData() //message sending place
    {
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        //---receiving Data from the Host----
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C# send info to Python

        //wheel angle 
        if(dataReceived == "getSteer") // check the python message, see if the security code meet the need
        {
            byte[] myWheelAng = Encoding.ASCII.GetBytes(wheelAng.ToString()); // convert target value to unicode and ready to send it to socket
            nwStream.Write(myWheelAng, 0, myWheelAng.Length); // send the unicode message to socket for python to read
        }
        

        //throttle position
        if(dataReceived == "getThrottle")
        {
            byte[] myThrottlePos = Encoding.ASCII.GetBytes(throttlePos.ToString());
            nwStream.Write(myThrottlePos, 0, myThrottlePos.Length);
        }

        //brake position
        if(dataReceived == "getBrake")
        {
            byte[] myBrakePos = Encoding.ASCII.GetBytes(brakePos.ToString());
            nwStream.Write(myBrakePos, 0, myBrakePos.Length);
        }

        //engine torque
        if(dataReceived == "getETorque")
        {
            byte[] myEngineTorque = Encoding.ASCII.GetBytes(engineTorque.ToString());
            nwStream.Write(myEngineTorque, 0, myEngineTorque.Length);
        }

        //brake torque
        if(dataReceived == "getBTorque")
        {
            byte[] myBrakeTorque = Encoding.ASCII.GetBytes(brakeTorque.ToString());
            nwStream.Write(myBrakeTorque, 0, myBrakeTorque.Length);
        }

        //engine rpm
        if(dataReceived == "getRPM")
        {
            byte[] myEngineRpm = Encoding.ASCII.GetBytes(engineRpm.ToString());
            nwStream.Write(myEngineRpm, 0, myEngineRpm.Length);
        }

        //simulation time, in game time
        if(dataReceived == "getSimTime")
        {
            byte[] mySimTime = Encoding.ASCII.GetBytes("simulation time " + simTime);
            nwStream.Write(mySimTime, 0, mySimTime.Length);
        }
        
        //absolute time, system time
        if(dataReceived == "getAbsTime")
        {
            byte[] myAbsTime = Encoding.ASCII.GetBytes("absolute time " + absTime);
            nwStream.Write(myAbsTime, 0, myAbsTime.Length);
        }

        //MPH of the car
        if(dataReceived == "getMPH")
        {
            byte[] myMph = Encoding.ASCII.GetBytes(mph.ToString());
            nwStream.Write(myMph, 0, myMph.Length);
        }

        //Position of the car
        if(dataReceived == "getVehPosX")
        {
            byte[] myVehPosX = Encoding.ASCII.GetBytes(vehPosX.ToString());
            nwStream.Write(myVehPosX, 0, myVehPosX.Length);
        }
        if(dataReceived == "getVehPosY")
        {
            byte[] myVehPosY = Encoding.ASCII.GetBytes(vehPosY.ToString());
            nwStream.Write(myVehPosY, 0, myVehPosY.Length);
        }
        if(dataReceived == "getVehPosZ")
        {
            byte[] myVehPosZ = Encoding.ASCII.GetBytes(vehPosZ.ToString());
            nwStream.Write(myVehPosZ, 0, myVehPosZ.Length);
        }

        //Car heading Angle
        if(dataReceived == "getVehHeading")
        {
            byte[] myVehAngle = Encoding.ASCII.GetBytes(vehAngle.ToString("f2"));
            nwStream.Write(myVehAngle, 0, myVehAngle.Length);
        }


        //Car Acceleration Lontitude
        if(dataReceived == "getVehAccLon")
        {
            byte[] myVehAccLon = Encoding.ASCII.GetBytes(vehAccLon.ToString("f3"));
            nwStream.Write(myVehAccLon, 0, myVehAccLon.Length);
        }

        //Acceleration in directions of the car
        if(dataReceived == "getVehAccX")
        {
            byte[] myVehAccX= Encoding.ASCII.GetBytes(vehAccX.ToString("f3"));
            nwStream.Write(myVehAccX, 0, myVehAccX.Length);
        }
        if(dataReceived == "getVehAccY")
        {
            byte[] myVehAccY = Encoding.ASCII.GetBytes(vehAccY.ToString("f3"));
            nwStream.Write(myVehAccY, 0, myVehAccY.Length);
        }
        if(dataReceived == "getVehAccZ")
        {
            byte[] myVehAccZ = Encoding.ASCII.GetBytes(vehAccZ.ToString("f3"));
            nwStream.Write(myVehAccZ, 0, myVehAccZ.Length);
        }

        // Car Collision State
        if(dataReceived == "getCollisionState")
        {
            byte[] myCarCollide = Encoding.ASCII.GetBytes(carInput.isCarCollided.ToString());
            nwStream.Write(myCarCollide, 0, myCarCollide.Length);
        }


        if(dataReceived == "getTurnSignal")
        {
            byte[] myTurnSignal = Encoding.ASCII.GetBytes(carInput.carTurnSignal.ToString());
            nwStream.Write(myTurnSignal, 0, myTurnSignal.Length);
        }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C# recieve info from Python + reset the set condition
/////////////////// SET

        // recieve info set Steering Wheel Angle
        if(dataReceived.Contains("setSteer"))
        {
            float setValue = float.Parse(dataReceived.Substring(9));

            carInput.overwrite_steer_wheel_angle_value = setValue; //assign values to the input
            carInput.overwrite_steer_wheel_angle = true;           //change the overwrite condition to true

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_Throttle_Pedal_Position
        if(dataReceived.Contains("setThrot"))
        {
            float setValue = float.Parse(dataReceived.Substring(9));

            carInput.overwrite_throttle_input_value = setValue;
            carInput.overwrite_throttle_input = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_Brake_Pedal_Position
        if(dataReceived.Contains("setBrake"))
        {
            float setValue = float.Parse(dataReceived.Substring(9));

            carInput.overwrite_brake_input_value = setValue;
            carInput.overwrite_brake_input = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_Vehicle_Speed
        if(dataReceived.Contains("setSpeed"))
        {
            int setValue = int.Parse(dataReceived.Substring(9));

            carInput.overwrite_mph_value = setValue;
            carInput.overwrite_mph = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);

        }
        

        //set_ Vehicle_Engine_Toque
        if(dataReceived.Contains("setETorque"))
        {
            float setValue = float.Parse(dataReceived.Substring(11));

            carInput.overwrite_engine_torque_value = setValue;
            carInput.overwrite_engine_torque = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_ Vehicle_Brake_Toque
        if(dataReceived.Contains("setBTorque"))
        {
            float setValue = float.Parse(dataReceived.Substring(11));

            carInput.overwrite_brake_torque_value = setValue;
            carInput.overwrite_brake_torque = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_Turn_Signal
        if(dataReceived.Contains("setTSignal"))
        {
            carInput.overwrite_turn_signal_value = int.Parse(dataReceived.Substring(11));
            carInput.overwrite_turn_signal = true;
            
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


        //set_Trigger_State
        if(dataReceived == "getFLWheelPosX")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(FLWheelX.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getFLWheelPosZ")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(FLWheelZ.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getFRWheelPosX")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(FRWheelX.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getFRWheelPosZ")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(FRWheelZ.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getRLWheelPosX")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(RLWheelX.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getRLWheelPosZ")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(RLWheelZ.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getRRWheelPosX")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(RRWheelX.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }
        if(dataReceived == "getRRWheelPosZ")
        {
            byte[] myWheelPos = Encoding.ASCII.GetBytes(RRWheelZ.ToString());
            nwStream.Write(myWheelPos, 0, myWheelPos.Length);
        }


        // set_Vehicle_Position
        if(dataReceived.Contains("setVehPosX"))
        {
            float setValue = float.Parse(dataReceived.Substring(11));
            setVehPosX = setValue;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        if(dataReceived.Contains("setVehPosY"))
        {
            
            float setValue = float.Parse(dataReceived.Substring(11));
            setVehPosY = setValue;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        if(dataReceived.Contains("setVehPosZ"))
        {
            
            float setValue = float.Parse(dataReceived.Substring(11));
            setVehPosZ = setValue;
            // change the condition to overwrite position
            setPos = true;

            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }


///////////////////

/////////////////// RESET

        
        // reset Steering Wheel Angle
        if(dataReceived == "reSteer")
        {
            carInput.overwrite_steer_wheel_angle = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }

        // reset Throttle_Pedal_Position
        if(dataReceived == "reThrot")
        {
            carInput.overwrite_throttle_input = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }

        // reset Brake_Pedal_Position
        if(dataReceived == "reBrake")
        {
            carInput.overwrite_brake_input = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        
        // reset Vehicle_Speed
        if(dataReceived == "reSpeed")
        {
            carInput.overwrite_mph = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        
        // reset Vehicle_Engine_Toque
        if(dataReceived == "reETorque")
        {
            carInput.overwrite_engine_torque = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        
        // reset Vehicle_Brake_Toque
        if(dataReceived == "reBTorque")
        {
            carInput.overwrite_brake_torque = false;
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }
        
        // reset Turn_Signal
        if(dataReceived == "reTSignal")
        {
            // indicator change cannot be make in another script, the actual indicator change script is in RCC_CarControllerV3.cs, 
            // in GetExternalInputs() sections
            carInput.reset_turn_signal = true;
            
            byte[] msgBack = Encoding.ASCII.GetBytes(dataReceived);
            nwStream.Write(msgBack, 0, msgBack.Length);
        }

///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (dataReceived != null)
        {
            //---Using received data---
            //receivedPos = StringToVector3(dataReceived); //<-- assigning receivedPos value from Python
            //print("received pos data, and moved the Cube!");

            //---Sending Data to Host----
            // byte[] myWriteBuffer = Encoding.ASCII.GetBytes("Hey I got your message Python! Do You see this massage?"); //Converting string to byte data
            // nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }

        //---Sending Data to Host----
        
    }

    public static Vector3 StringToVector3(string sVector) //use for converting string to vector
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');

        // store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[1]),
            float.Parse(sArray[2]));

        return result;
    }
    /*
    public static string GetLocalIPAddress()
    {
        var host = Dns.GetHostEntry(Dns.GetHostName());
        foreach (var ip in host.AddressList)
        {
            if (ip.AddressFamily == AddressFamily.InterNetwork)
            {
                return ip.ToString();
            }
        }
        throw new System.Exception("No network adapters with an IPv4 address in the system!");
    }
    */
}