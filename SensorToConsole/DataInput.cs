using System;
using System.IO.Ports;
using System.Threading;
using System.IO;


namespace SensorToConsole

{
    class DataInput
    {
        //Variables for output values
        //Hands
        public string WX_RX, WY_RX, WZ_RX;
        public string TX_RX, TY_RX, TZ_RX;
        public string IX_RX, IY_RX, IZ_RX;
        public string MX_RX, MY_RX, MZ_RX;
        public string LOSS_HRX;

        public string WX_LX, WY_LX, WZ_LX;
        public string TX_LX, TY_LX, TZ_LX;
        public string IX_LX, IY_LX, IZ_LX;
        public string MX_LX, MY_LX, MZ_LX;
        public string LOSS_HLX;


        public string ROLLW_LX, ROLLT_LX, ROLLI_LX, ROLLM_LX;
        public string PITCHW_LX, PITCHT_LX, PITCHI_LX, PITCHM_LX;
        public string YAWW_LX, YAWT_LX, YAWI_LX, YAWM_LX;
        public string LOSS_HLX_AHRS;

        public string ROLLW_RX, ROLLT_RX, ROLLI_RX, ROLLM_RX;
        public string PITCHW_RX, PITCHT_RX, PITCHI_RX, PITCHM_RX;
        public string YAWW_RX, YAWT_RX, YAWI_RX, YAWM_RX;
        public string LOSS_HRX_AHRS;

        public string ROLLF_RX, PITCHF_RX, YAWF_RX, LOSS_FRX_AHRS;
        public string ROLLF_LX, PITCHF_LX, YAWF_LX, LOSS_FLX_AHRS;

        //Foots
        public string FX_RX, FY_RX, FZ_RX, LOSS_FRX;
        public string FX_LX, FY_LX, FZ_LX, LOSS_FLX;


        int workMode;
        string comName;
        bool DATA_GRAPH = false; bool DATA_LOG = false; bool DATA_AHRS = false; bool DATA_VM = false;
        bool DATA_HAND_LX = false; bool DATA_HAND_RX = false; bool DATA_FOOT_LX = false; bool DATA_FOOT_RX = false;
        bool accelerometer = true; bool gyroscope = false; bool magnetometer = false;

        bool connected = false;
        bool rec = false;
        bool COM_SELECTED = false; bool COM_OPEN = false;
        bool DISCOVERY = false; bool IDs_FIND = false;
        bool hand_lx_connecting = false; bool hand_lx_disconnecting = false;
        bool foot_lx_connecting = false; bool foot_lx_disconnecting = false;
        bool hand_rx_connecting = false; bool hand_rx_disconnecting = false;
        bool foot_rx_connecting = false; bool foot_rx_disconnecting = false;
        bool hand_rx_waiting_for_connection = false; bool hand_rx_waiting_for_disconnection = false;
        bool foot_lx_waiting_for_connection = false; bool foot_lx_waiting_for_disconnection = false;
        bool foot_rx_waiting_for_connection = false; bool foot_rx_waiting_for_disconnection = false;
        bool CONNECTION_STEP1 = false; bool CONNECTION_STEP2 = false; bool CONNECTION_STEP3 = false;
        bool RESET_STOP = false; bool RESET = false;
        bool PACKET_GL_COMPLETED = false; bool PACKET_FL_COMPLETED = false;
        bool PACKET_GR_COMPLETED = false; bool PACKET_FR_COMPLETED = false;
        bool Calibration = false;
        bool path_existHLX = false; bool path_existFLX = false;
        bool path_existHRX = false; bool path_existFRX = false;
        bool DataLossControlLG = false; bool DataLossControlLF = false;
        bool DataLossControlRG = false; bool DataLossControlRF = false;
        byte[] GLOVE_PACKET_LX = new byte[79]; byte[] FOOT_PACKET_LX = new byte[25];
        byte[] GLOVE_PACKET_RX = new byte[79]; byte[] FOOT_PACKET_RX = new byte[25];
        int working_mode = 0;
        int count_chartACC = 0; int count_chartGYRO = 0; int count_chartMAG = 0;
        int count_timeout_discovery = 0;
        int hand_lx_connecting_count = 0; int hand_lx_disconnecting_count = 0;
        int foot_lx_connecting_count = 0; int foot_lx_disconnecting_count = 0;
        int hand_rx_connecting_count = 0; int hand_rx_disconnecting_count = 0;
        int foot_rx_connecting_count = 0; int foot_rx_disconnecting_count = 0;
        int ERROR_LG = 0; int ERROR_LF = 0;
        int ERROR_RG = 0; int ERROR_RF = 0;
        int INDEX_GL = 0; int INDEX_FL = 0;
        int INDEX_GR = 0; int INDEX_FR = 0;
        int PACKET_NUM_G = 79; int PACKET_NUM_F = 25;
        int ERROR_LG1 = 0; int ERROR_LG2 = 0; int ERROR_LG3 = 0; int ERROR_LG4 = 0; int ERROR_LG5 = 0;
        int ERROR_LG6 = 0; int ERROR_LG7 = 0; int ERROR_LG8 = 0; int ERROR_LG9 = 0;
        int ERROR_LF1 = 0; int ERROR_LF2 = 0; int ERROR_LF3 = 0; int ERROR_LF4 = 0; int ERROR_LF5 = 0;
        int ERROR_LF6 = 0; int ERROR_LF7 = 0; int ERROR_LF8 = 0; int ERROR_LF9 = 0;
        int ERROR_RG1 = 0; int ERROR_RG2 = 0; int ERROR_RG3 = 0; int ERROR_RG4 = 0; int ERROR_RG5 = 0;
        int ERROR_RG6 = 0; int ERROR_RG7 = 0; int ERROR_RG8 = 0; int ERROR_RG9 = 0;
        int ERROR_RF1 = 0; int ERROR_RF2 = 0; int ERROR_RF3 = 0; int ERROR_RF4 = 0; int ERROR_RF5 = 0;
        int ERROR_RF6 = 0; int ERROR_RF7 = 0; int ERROR_RF8 = 0; int ERROR_RF9 = 0;
        int countMicroGL; int[] countMicroLG = new int[2]; int countMicroLossLG = 0; int frequencyLG = 0; int countfreqLG = 0;
        int countMicroGR; int[] countMicroRG = new int[2]; int countMicroLossRG = 0; int frequencyRG = 0; int countfreqRG = 0;
        int[] LineLossLG = new int[50]; int LossIndexLG = 0; int[] vectLossLG = new int[50];
        int countMicroFL = 0; int[] countMicroLF = new int[2]; int countMicroLossLF = 0; int frequencyLF = 0; int countfreqLF = 0;
        int countMicroFR = 0; int[] countMicroRF = new int[2]; int countMicroLossRF = 0; int frequencyRF = 0; int countfreqRF = 0;
        int[] LineLossLF = new int[50]; int LossIndexLF = 0; int[] vectLossLF = new int[50];
        int data_mode_update = 0;
        char FOOT_NODE_LX, GLOVE_NODE_LX;
        char FOOT_NODE_RX, GLOVE_NODE_RX;
        float accWX_LX, accWY_LX, accWZ_LX; float accTX_LX, accTY_LX, accTZ_LX; float accIX_LX, accIY_LX, accIZ_LX; float accMX_LX, accMY_LX, accMZ_LX;
        float gyrWX_LX, gyrWY_LX, gyrWZ_LX; float gyrTX_LX, gyrTY_LX, gyrTZ_LX; float gyrIX_LX, gyrIY_LX, gyrIZ_LX; float gyrMX_LX, gyrMY_LX, gyrMZ_LX;
        float magWX_LX, magWY_LX, magWZ_LX; float magTX_LX, magTY_LX, magTZ_LX; float magIX_LX, magIY_LX, magIZ_LX; float magMX_LX, magMY_LX, magMZ_LX;
        float accxF_LX, accyF_LX, acczF_LX; float gyrxF_LX, gyryF_LX, gyrzF_LX; float magxF_LX, magyF_LX, magzF_LX;
        float accWX_RX, accWY_RX, accWZ_RX; float accTX_RX, accTY_RX, accTZ_RX; float accIX_RX, accIY_RX, accIZ_RX; float accMX_RX, accMY_RX, accMZ_RX;
        float gyrWX_RX, gyrWY_RX, gyrWZ_RX; float gyrTX_RX, gyrTY_RX, gyrTZ_RX; float gyrIX_RX, gyrIY_RX, gyrIZ_RX; float gyrMX_RX, gyrMY_RX, gyrMZ_RX;
        float magWX_RX, magWY_RX, magWZ_RX; float magTX_RX, magTY_RX, magTZ_RX; float magIX_RX, magIY_RX, magIZ_RX; float magMX_RX, magMY_RX, magMZ_RX;
        float accxF_RX, accyF_RX, acczF_RX; float gyrxF_RX, gyryF_RX, gyrzF_RX; float magxF_RX, magyF_RX, magzF_RX;
        float rollW_LX, rollT_LX, rollI_LX, rollM_LX;
        float rollW_RX, rollT_RX, rollI_RX, rollM_RX;
        float pitchW_LX, pitchT_LX, pitchI_LX, pitchM_LX;
        float pitchW_RX, pitchT_RX, pitchI_RX, pitchM_RX;
        float yawW_LX, yawT_LX, yawI_LX, yawM_LX;
        float yawW_RX, yawT_RX, yawI_RX, yawM_RX;
        float rollF_LX, pitchF_LX, yawF_LX;
        float rollF_RX, pitchF_RX, yawF_RX;

        SerialPort SP1 = new SerialPort();
        int seconds = 0;
        private Thread COMreceiver;
        string pathCOM, pathID;
        string ID_GLOVE_LX, ID_FOOT_LX, ID_GLOVE_RX, ID_FOOT_RX, ID_discovered_list;
        string ID_READ;
        string starting_pathTXT, report;
        string FileNameHLX, pathHLX, FileNameFLX, pathFLX;
        string FileNameHRX, pathHRX, FileNameFRX, pathFRX;
        string accWX_LX_txt, accWY_LX_txt, accWZ_LX_txt, accTX_LX_txt, accTY_LX_txt, accTZ_LX_txt, accIX_LX_txt, accIY_LX_txt, accIZ_LX_txt, accMX_LX_txt, accMY_LX_txt, accMZ_LX_txt;
        string gyrWX_LX_txt, gyrWY_LX_txt, gyrWZ_LX_txt, gyrTX_LX_txt, gyrTY_LX_txt, gyrTZ_LX_txt, gyrIX_LX_txt, gyrIY_LX_txt, gyrIZ_LX_txt, gyrMX_LX_txt, gyrMY_LX_txt, gyrMZ_LX_txt;
        string magWX_LX_txt, magWY_LX_txt, magWZ_LX_txt, magTX_LX_txt, magTY_LX_txt, magTZ_LX_txt, magIX_LX_txt, magIY_LX_txt, magIZ_LX_txt, magMX_LX_txt, magMY_LX_txt, magMZ_LX_txt;
        string rollW_LX_txt, pitchW_LX_txt, yawW_LX_txt, rollT_LX_txt, pitchT_LX_txt, yawT_LX_txt, rollI_LX_txt, pitchI_LX_txt, yawI_LX_txt, rollM_LX_txt, pitchM_LX_txt, yawM_LX_txt;
        string accxF_LX_txt, accyF_LX_txt, acczF_LX_txt, gyrxF_LX_txt, gyryF_LX_txt, gyrzF_LX_txt, magxF_LX_txt, magyF_LX_txt, magzF_LX_txt;
        string rollF_LX_txt, pitchF_LX_txt, yawF_LX_txt;
        string accWX_RX_txt, accWY_RX_txt, accWZ_RX_txt, accTX_RX_txt, accTY_RX_txt, accTZ_RX_txt, accIX_RX_txt, accIY_RX_txt, accIZ_RX_txt, accMX_RX_txt, accMY_RX_txt, accMZ_RX_txt;
        string gyrWX_RX_txt, gyrWY_RX_txt, gyrWZ_RX_txt, gyrTX_RX_txt, gyrTY_RX_txt, gyrTZ_RX_txt, gyrIX_RX_txt, gyrIY_RX_txt, gyrIZ_RX_txt, gyrMX_RX_txt, gyrMY_RX_txt, gyrMZ_RX_txt;
        string magWX_RX_txt, magWY_RX_txt, magWZ_RX_txt, magTX_RX_txt, magTY_RX_txt, magTZ_RX_txt, magIX_RX_txt, magIY_RX_txt, magIZ_RX_txt, magMX_RX_txt, magMY_RX_txt, magMZ_RX_txt;
        string rollW_RX_txt, pitchW_RX_txt, yawW_RX_txt, rollT_RX_txt, pitchT_RX_txt, yawT_RX_txt, rollI_RX_txt, pitchI_RX_txt, yawI_RX_txt, rollM_RX_txt, pitchM_RX_txt, yawM_RX_txt;
        string accxF_RX_txt, accyF_RX_txt, acczF_RX_txt, gyrxF_RX_txt, gyryF_RX_txt, gyrzF_RX_txt, magxF_RX_txt, magyF_RX_txt, magzF_RX_txt;
        string rollF_RX_txt, pitchF_RX_txt, yawF_RX_txt;


        //-------------------------------------------------------------------------------
        //Set up serial port(called in FormLoad()) 
        public void SystemInit()
        {
            SP1 = new SerialPort(); SP1.Parity = Parity.None;
            SP1.StopBits = StopBits.One; SP1.DataBits = 8;
            SP1.ReadBufferSize = 4096; SP1.ReadTimeout = 4000;
            SP1.WriteBufferSize = 4096; SP1.WriteTimeout = 1000;
            SP1.BaudRate = 460800; SP1.ReceivedBytesThreshold = 4096;

            starting_pathTXT = @"c:\HANDi\DATA";
            if (!Directory.Exists(starting_pathTXT))
            {
                System.IO.Directory.CreateDirectory(@"c:\HANDi\DATA").ToString();
            }
        }

        //-------------------------------------------------------------------------------
        //setting up app
        public void FormLoad()
        {
            SystemInit();
            string[] COMport = SerialPort.GetPortNames();
            
            if (File.Exists(@"c:\HANDi\COM\DEFAULT_COM.txt"))
            {
                pathCOM = @"c:\HANDi\COM\DEFAULT_COM.txt";
                COM_SELECTED = true;
                string[] COM = new string[1];
                COM = File.ReadAllLines(pathCOM);
                SP1.PortName = COM[0]; Thread.Sleep(500); 
                try
                {
                    try
                    {
                        SP1.Open(); COM_OPEN = true;
                    }
                    catch (IOException) { COM_OPEN = false; Console.WriteLine("COM not found! Be sure of dongle connection!"); }
                }
                catch (UnauthorizedAccessException)
                {
                    COM_OPEN = false; Console.WriteLine("COM ALREADY IN USE!");
                }
            }
            if (File.Exists(@"c:\HANDi\ID_LIST\DEFAULT_ID_LIST.txt"))
            {
                pathID = @"c:\HANDi\ID_LIST\DEFAULT_ID_LIST.txt";
                IDs_FIND = true;
                string[] ALL_ID = new string[4];
                ALL_ID = File.ReadAllLines(pathID); ID_FOOT_LX = ALL_ID[0]; ID_GLOVE_LX = ALL_ID[1]; ID_FOOT_RX = ALL_ID[2]; ID_GLOVE_RX = ALL_ID[3];
            }
            else { IDs_FIND = false; }

            COMreceiver = new Thread(new ThreadStart(COMRead));
            COMreceiver.Start();
        }

        //-------------------------------------------------------------------------------
        //Disconecting sensors
        public void AppClose()
        {
            if (connected)
            {
                Thread.Sleep(4000);
                Console.WriteLine("System disconnection");
                Thread.Sleep(4000);
            }

            try { SP1.Close(); }
            catch { }

            COMreceiver.Abort(); ;
            while (SP1.IsOpen)
            { }
        }

        //-------------------------------------------------------------------------------
        //Save com name to file 
        public void SaveCom()
        {
            //There is picking which COM will be saved
            Console.WriteLine("Aviable ports:");
            string[] COMport = SerialPort.GetPortNames();
            foreach (string car in COMport)
            {
                Console.WriteLine(car);
            }
            Console.WriteLine("Please pick com which you want to save! ");
            string comName = Console.ReadLine();

            COM_SELECTED = true;

            Console.WriteLine("You picked and saved this com : " + comName);

            try
             {
                if (!COM_OPEN)
                {
                    if (COM_SELECTED)
                    {
                        SP1.PortName = comName;
                        try
                        {
                            SP1.Open(); COM_OPEN = true;
                        }
                        catch (IOException) { COM_OPEN = false; Console.WriteLine("COM not found! Be sure of dongle connection!"); }
                        if (!Directory.Exists(@"c:\HANDi\COM\")) { System.IO.Directory.CreateDirectory(@"c:\HANDi\COM\").ToString(); }
                        pathCOM = System.IO.Path.Combine(@"c:\HANDi\COM\", "DEFAULT_COM.txt");
                        if ((!File.Exists(pathCOM)) && (COM_OPEN))
                        {

                            StreamWriter Nome_fileCOM = new StreamWriter(pathCOM);
                            Nome_fileCOM.Close();
                            StreamWriter fileDatiCOM = File.AppendText(pathCOM);
                            fileDatiCOM.Flush();
                            fileDatiCOM.WriteLine(SP1.PortName.ToString());
                            fileDatiCOM.Flush();
                            fileDatiCOM.Close();

                        }
                    }
                    else
                    { Console.WriteLine("SELECT BEFORE THE PROPER COM PORT!"); }
                }
            }
            catch (UnauthorizedAccessException)
            { COM_OPEN = false; Console.WriteLine("COM ALREADY IN USE!"); }
        }

        //-------------------------------------------------------------------------------
        //Delete saved com file
        public void DeleteCom()
        {
            if (COM_OPEN)
            {
                SP1.Close(); COM_OPEN = false;
            }
            if (File.Exists(@"c:\HANDi\COM\DEFAULT_COM.txt"))
            {
                File.Delete(@"c:\HANDi\COM\DEFAULT_COM.txt");
            }
        }

        //-------------------------------------------------------------------------------
        //Pick aplication mode LOG or AHRS
        public void PickAplication()
        {
            int datalog;
            DATA_AHRS = false;
            DATA_LOG = false;
            Console.WriteLine("Please pick data mode. Options:");
            Console.WriteLine("0. DATA_AHRS");
            Console.WriteLine("1. DATA_LOG");
            datalog = Convert.ToInt32(Console.ReadLine());

            switch (datalog)
            {
                case 0:
                    DATA_AHRS = true;
                    break;
                case 1:
                    DATA_LOG = true;
                    break;
                default:
                    DATA_LOG = true;
                    Console.WriteLine("DATA_LOG was picked as default.");
                    break;
            }
        }

        //-------------------------------------------------------------------------------
        //Pick mode between acc, gyr and mag
        public void PickMeter()
        {
            int meter;
            accelerometer = false;
            gyroscope = false;
            magnetometer = false;

            Console.WriteLine("Please pick data mode. Options:");
            Console.WriteLine("0. Accelerometer");
            Console.WriteLine("1. Gysroscope");
            Console.WriteLine("2. Magnetoneter");
            meter = Convert.ToInt32(Console.ReadLine());

            switch (meter)
            {
                case 1:
                    accelerometer = true;
                    break;
                case 2:
                    gyroscope = true;
                    break;
                case 3:
                    magnetometer = true;
                    break;
                default:
                    accelerometer = true;
                    Console.WriteLine("Accelerometer was picked as default.");
                    break;
            }

        }

        //-------------------------------------------------------------------------------
        //Pick which limb you want
        public void PickLimb(int limb)
        {

            DATA_HAND_LX = false;
            DATA_HAND_RX = false;
            DATA_FOOT_LX = false;
            DATA_FOOT_RX = false;
            switch (limb)
            {
                case 1:
                    DATA_HAND_LX = true;
                    break;
                case 2:
                    DATA_HAND_RX = true;
                    break;
                case 3:
                    DATA_FOOT_LX = true;
                    break;
                case 4:
                    DATA_FOOT_RX = true;
                    break;
                default:
                    DATA_HAND_RX = true;
                    break;
            }
        }

        //-------------------------------------------------------------------------------
        //Connect sensor or disconect
        //Can be used like button
        public void ConnectSensor(int selectIndex)
        {
            if (COM_OPEN)
            {
                if (IDs_FIND)
                {
                    if (!connected)
                    {
                        switch (selectIndex)
                        {
                            case 0: //HAND_LX
                                hand_lx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");

                                break;
                            case 1: //HAND_RX
                                hand_rx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_RX + "\r\n");
                                break;
                            case 2://FOOT_LX
                                foot_lx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_FOOT_LX + "\r\n");
                                break;
                            case 3://FOOT_RX
                                foot_rx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_FOOT_RX + "\r\n");
                                break;
                            case 4://BOTH_HAND
                                   //VM_APP_ENABLE();
                                hand_lx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");
                                hand_rx_waiting_for_connection = true;
                                break;
                            case 5://BOTH_FOOT
                                foot_lx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_FOOT_LX + "\r\n");
                                foot_rx_waiting_for_connection = true;
                                break;
                            case 6://HAND_FOOT_LX
                                hand_lx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");
                                foot_lx_waiting_for_connection = true;
                                break;
                            case 7://HAND_FOOT_RX
                                hand_rx_connecting = true;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_RX + "\r\n");
                                foot_rx_waiting_for_connection = true;
                                break;
                        }

                    }


                    //function for disconecting
                    else if (connected && (!rec))
                    {
                        switch (selectIndex)
                        {
                            case 0: //HAND_LX
                                hand_lx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 1: //HAND_RX
                                hand_rx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 2://FOOT_LX
                                foot_lx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 3://FOOT_RX
                                foot_rx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 4: //BOTH_HAND
                                    //VM_APP_ENABLE();
                                hand_lx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                hand_rx_waiting_for_disconnection = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 5: //BOTH_FOOT
                                foot_lx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                foot_rx_waiting_for_disconnection = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 6: //HAND_FOOT_LX
                                hand_lx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                foot_lx_waiting_for_disconnection = true;
                                SP1.Write("^#^$^%");
                                break;
                            case 7: //HAND_FOOT_RX
                                hand_rx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                foot_rx_waiting_for_disconnection = true;
                                SP1.Write("^#^$^%");
                                break;
                        }
                    }
                }
                else
                {
                    Console.WriteLine("ID list not found!");
                }
            }
            else Console.WriteLine("COM port closed!");
        }

        //-------------------------------------------------------------------------------
        //Start and end recording
        //Can be used like button
        public void Rec(int selectIndex)
        {
            if (connected)
            {
                if (DATA_LOG || DATA_AHRS)
                {
                    if (!rec)
                    {
                        SP1.DiscardInBuffer();
                        rec = true; seconds = 0;
                        char[] Data = new char[7];
                        switch (selectIndex)
                        {
                            case 0://HAND_LX
                                countfreqLG = 0; countMicroLossLG = 0;

                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 0;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 0;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                }
                                SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                break;
                            case 1://HAND_RX
                                countfreqRG = 0; countMicroLossRG = 0;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 1;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 1;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                }

                                SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                break;
                            case 2://FOOT_LX
                                countfreqLF = 0; countMicroLossLF = 0;
                                TXT_SAVE();
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 2;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 2;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                }
                                SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                break;
                            case 3://FOOT_RX
                                countfreqRF = 0; countMicroLossRF = 0;

                                TXT_SAVE();

                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 3;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 3;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                }
                                SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                break;
                            case 4://BOTH_HAND
                                countfreqLG = 0; countfreqRG = 0;
                                countMicroLossLG = 0; countMicroLossRG = 0;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 4;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 4;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }

                                break;
                            case 5://BOTH_FOOT
                                countfreqLF = 0; countfreqRF = 0;
                                countMicroLossLF = 0; countMicroLossRF = 0;


                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 5;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'R';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 5;
                                    TXT_SAVE();
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }

                                break;
                            case 6://HAND_FOOT_LX
                                countfreqLF = 0; countfreqLG = 0;
                                countMicroLossLG = 0; countMicroLossLF = 0;

                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 6;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 6;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }

                                break;
                            case 7://HAND_FOOT_RX
                                countfreqRF = 0; countfreqRG = 0;
                                countMicroLossRG = 0; countMicroLossRF = 0;


                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    working_mode = 7;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'N'; Data[5] = 'O';
                                    Data[6] = 'L';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }
                                else if (DATA_AHRS)
                                {
                                    working_mode = 7;
                                    TXT_SAVE();
                                    Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                    Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                    Data[2] = '0'; Data[3] = '3';
                                    Data[4] = 'E'; Data[5] = 'K';
                                    Data[6] = 'F';
                                    SP1.Write(Data, 0, 7); Thread.Sleep(5);
                                }

                                break;
                        }
                    }


                    //stop rec function
                    else
                    {
                        SP1.DiscardInBuffer();
                        rec = false;

                        COMreceiver = new Thread(new ThreadStart(COMRead));
                        COMreceiver.Start();
                        char[] Data = new char[7];
                        switch (selectIndex)
                        {
                            case 0://HAND_LX
                                DataLossControlLG = false;
                                Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                break;
                            case 1://HAND_RX
                                DataLossControlRG = false;
                                Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                break;
                            case 2: //FOOT_LX
                                DataLossControlLF = false;
                                Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                break;
                            case 3: //FOOT_RX
                                DataLossControlRF = false;
                                Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                break;
                            case 4://BOTH_HAND
                                DataLossControlLG = false;
                                DataLossControlRG = false;
                                Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                break;
                            case 5://BOTH_FOOT
                                DataLossControlLF = false;
                                DataLossControlRF = false;
                                Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                break;
                            case 6://HAND_FOOT_LX
                                DataLossControlLG = false;
                                DataLossControlLF = false;
                                Data[0] = GLOVE_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                Data[0] = FOOT_NODE_LX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                break;
                            case 7://HAND_FOOT_RX
                                DataLossControlRG = false;
                                DataLossControlRF = false;
                                Data[0] = GLOVE_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                Data[0] = FOOT_NODE_RX; Data[1] = '0';
                                Data[2] = '0'; Data[3] = '3';
                                Data[4] = 'S'; Data[5] = 'T';
                                Data[6] = 'P'; SP1.Write(Data, 0, 7);
                                Thread.Sleep(5);
                                break;
                        }

                    }
                }
                else Console.WriteLine("Select the application!");
            }
            else Console.WriteLine("Connect at least one sensor node!");
        }


        //-------------------------------------------------------------------------------
        //This will show txtfile in WinForms
        public void Txt(int modePick)
        {
            switch (modePick)
            {
                case 0://HAND_LX
                    System.Diagnostics.Process.Start(pathHLX);
                    break;
                case 1://HAND_RX
                    System.Diagnostics.Process.Start(pathHRX);
                    break;
                case 2://FOOT_LX
                    System.Diagnostics.Process.Start(pathFLX);
                    break;
                case 3://FOOT_RX
                    System.Diagnostics.Process.Start(pathFRX);
                    break;
                case 4://BOTH_HAND
                    System.Diagnostics.Process.Start(pathHLX);
                    System.Diagnostics.Process.Start(pathHRX);
                    break;
                case 5://BOTH_FOOT
                    System.Diagnostics.Process.Start(pathFLX);
                    System.Diagnostics.Process.Start(pathFRX);
                    break;
                case 6://HAND_FOOT_LX
                    System.Diagnostics.Process.Start(pathHLX);
                    System.Diagnostics.Process.Start(pathFLX);
                    break;
                case 7://HAND_FOOT_RX
                    System.Diagnostics.Process.Start(pathHRX);
                    System.Diagnostics.Process.Start(pathFRX);
                    break;
            }
        }


        //-------------------------------------------------------------------------------
        //Function for prin out data. This is called in updatePanel()
        //This print DATA mode
        public void printHandData(string side)
        {
            if (side == "R")
            {
                Console.WriteLine("{0} {1} {2}", WX_RX, WY_RX, WZ_RX);
                Console.WriteLine("{0} {1} {2}", TX_RX, TY_RX, TZ_RX);
                Console.WriteLine("{0} {1} {2}", IX_RX, IY_RX, IZ_RX);
                Console.WriteLine("{0} {1} {2}", MX_RX, MY_RX, MZ_RX);
                Console.WriteLine("{0} ", LOSS_HRX);
            }
            else if (side == "L")
            {
                Console.WriteLine("{0} {1} {2}", WX_LX, WY_LX, WZ_LX);
                Console.WriteLine("{0} {1} {2}", TX_LX, TY_LX, TZ_LX);
                Console.WriteLine("{0} {1} {2}", IX_LX, IY_LX, IZ_LX);
                Console.WriteLine("{0} {1} {2}", MX_LX, MY_LX, MZ_LX);
                Console.WriteLine("{0} ", LOSS_HLX);
            }
        }


        //-------------------------------------------------------------------------------
        //This print AHRS data
        public void printHandDataAHRS(string side)
        {
            if (side == "R")
            {
                Console.WriteLine("{0} {1} {2} {3}", ROLLW_RX, ROLLT_RX, ROLLI_RX, ROLLM_RX);
                Console.WriteLine("{0} {1} {2} {3}", PITCHW_RX, PITCHT_RX, PITCHI_RX, PITCHM_RX);
                Console.WriteLine("{0} {1} {2} {3}", YAWW_RX, YAWT_RX, YAWI_RX, YAWM_RX);
                Console.WriteLine("{0} ", LOSS_HRX_AHRS);
            }
            else if (side == "L")
            {
                Console.WriteLine("{0} {1} {2} {3}", ROLLW_LX, ROLLT_LX, ROLLI_LX, ROLLM_LX);
                Console.WriteLine("{0} {1} {2} {3}", PITCHW_LX, PITCHT_LX, PITCHI_LX, PITCHM_LX);
                Console.WriteLine("{0} {1} {2} {3}", YAWW_LX, YAWT_LX, YAWI_LX, YAWM_LX);
                Console.WriteLine("{0} ", LOSS_HLX_AHRS);
            }
        }


        //-------------------------------------------------------------------------------
        //This print foot DATA 
        public void printFootData(string side)
        {
            string[] RXOutput = { FX_RX, FY_RX, FZ_RX, LOSS_FRX };
            string[] LXOutput = { FX_LX, FY_LX, FZ_LX, LOSS_FLX };
            if (side == "R")
            {
                foreach (var item in RXOutput)
                {
                    Console.WriteLine("\r{0}", item.ToString());
                }
            }
            else if (side == "L")
            {
                foreach (var item in LXOutput)
                {
                    Console.WriteLine("\r{0}", item.ToString());
                }
            }
        }


        //-------------------------------------------------------------------------------
        //This print foot AHRS 
        public void printFootDataAHRS(string side)
        {
            string[] RXOutput = { ROLLF_RX, PITCHF_RX, YAWF_RX, LOSS_FRX_AHRS };
            string[] LXOutput = { ROLLF_LX, PITCHF_LX, YAWF_LX, LOSS_FLX_AHRS };
            if (side == "R")
            {
                foreach (var item in RXOutput)
                {
                    Console.WriteLine("\r{0}", item.ToString());
                }
            }
            else if (side == "L")
            {
                foreach (var item in LXOutput)
                {
                    Console.WriteLine("\r{0}", item.ToString());
                }
            }
        }


        //-------------------------------------------------------------------------------
        //There you will pick workmode
        public void PickWorkmode()
        {
            string[] modes = { "Left hand", "Right hand", "Left foot", "Rigth foot", "Both hand", "Both foot", "Left hand and foot", "Right hand and foot" };
            Console.WriteLine("Please chose working mode. Options : ");
            int i = 0;
            foreach (string item in modes)
            {
                Console.WriteLine(i + " " + item);
                i++;
            }
            workMode = Convert.ToInt32(Console.ReadLine());
            Console.WriteLine("You have picked " + (int)workMode);
        }


        public delegate void UpdateGUI();
        //-------------------------------------------------------------------------------
        //There are main visualization of data 
        public void UpdatePanel(int workmode)
        {
            if (rec)
            {
                //Picking mode
                if (DATA_LOG || DATA_AHRS)
                {
                    switch (workmode)
                    {
                        case 0:/*HAND_LX*/
                            if (DATA_LOG)
                            {
                                if (accelerometer) data_mode_update = 0;
                                else if (gyroscope) data_mode_update = 1;
                                else if (magnetometer) data_mode_update = 2;
                            }
                            else if (DATA_AHRS) data_mode_update = 3;
                            break;
                        case 1:/*HAND_RX*/
                            if (DATA_LOG)
                            {
                                if (accelerometer) data_mode_update = 8;
                                else if (gyroscope) data_mode_update = 9;
                                else if (magnetometer) data_mode_update = 10;
                            }
                            else if (DATA_AHRS) data_mode_update = 11;
                            break;
                        case 2: /*FOOT_LX*/
                            if (DATA_LOG)
                            {
                                if (accelerometer) data_mode_update = 4;
                                else if (gyroscope) data_mode_update = 5;
                                else if (magnetometer) data_mode_update = 6;
                            }
                            else if (DATA_AHRS) data_mode_update = 7;
                            break;
                        case 3: /*FOOT_RX*/
                            if (DATA_LOG)
                            {
                                if (accelerometer) data_mode_update = 12;
                                else if (gyroscope) data_mode_update = 13;
                                else if (magnetometer) data_mode_update = 14;
                            }
                            else if (DATA_AHRS) data_mode_update = 15;
                            break;
                        case 4:/*BOTH_HAND*/
                            if (DATA_LOG)
                            {
                                if ((accelerometer) && (DATA_HAND_LX)) data_mode_update = 0;
                                else if ((gyroscope) && (DATA_HAND_LX)) data_mode_update = 1;
                                else if ((magnetometer) && (DATA_HAND_LX)) data_mode_update = 2;
                                else if ((accelerometer) && (DATA_HAND_RX)) data_mode_update = 8;
                                else if ((gyroscope) && (DATA_HAND_RX)) data_mode_update = 9;
                                else if ((magnetometer) && (DATA_HAND_RX)) data_mode_update = 10;
                            }
                            else if ((DATA_AHRS) && (DATA_HAND_LX)) data_mode_update = 3;
                            else if ((DATA_AHRS) && (DATA_HAND_RX)) data_mode_update = 11;
                            break;
                        case 5:/*BOTH_FOOT*/
                            if (DATA_LOG)
                            {
                                if ((accelerometer) && (DATA_FOOT_LX)) data_mode_update = 4;
                                else if ((gyroscope) && (DATA_FOOT_LX)) data_mode_update = 5;
                                else if ((magnetometer) && (DATA_FOOT_LX)) data_mode_update = 6;
                                else if ((accelerometer) && (DATA_FOOT_RX)) data_mode_update = 12;
                                else if ((gyroscope) && (DATA_FOOT_RX)) data_mode_update = 13;
                                else if ((magnetometer) && (DATA_FOOT_RX)) data_mode_update = 14;
                            }
                            else if ((DATA_AHRS) && (DATA_FOOT_LX)) data_mode_update = 7;
                            else if ((DATA_AHRS) && (DATA_FOOT_RX)) data_mode_update = 15;
                            break;
                        case 6:/*HAND_FOOT_LX*/
                            if (DATA_LOG)
                            {
                                if ((accelerometer) && (DATA_HAND_LX)) data_mode_update = 0;
                                else if ((gyroscope) && (DATA_HAND_LX)) data_mode_update = 1;
                                else if ((magnetometer) && (DATA_HAND_LX)) data_mode_update = 2;
                                else if ((accelerometer) && (DATA_FOOT_LX)) data_mode_update = 4;
                                else if ((gyroscope) && (DATA_FOOT_LX)) data_mode_update = 5;
                                else if ((magnetometer) && (DATA_FOOT_LX)) data_mode_update = 6;
                            }
                            else if ((DATA_AHRS) && (DATA_HAND_LX)) data_mode_update = 3;
                            else if ((DATA_AHRS) && (DATA_FOOT_LX)) data_mode_update = 7;
                            break;
                        case 7:/*HAND_FOOT_RX*/
                            if (DATA_LOG)
                            {
                                if ((accelerometer) && (DATA_HAND_RX)) data_mode_update = 8;
                                else if ((gyroscope) && (DATA_HAND_RX)) data_mode_update = 9;
                                else if ((magnetometer) && (DATA_HAND_RX)) data_mode_update = 10;
                                else if ((accelerometer) && (DATA_FOOT_RX)) data_mode_update = 12;
                                else if ((gyroscope) && (DATA_FOOT_RX)) data_mode_update = 13;
                                else if ((magnetometer) && (DATA_FOOT_RX)) data_mode_update = 14;
                            }
                            else if ((DATA_AHRS) && (DATA_HAND_RX)) data_mode_update = 11;
                            else if ((DATA_AHRS) && (DATA_FOOT_RX)) data_mode_update = 15;
                            break;
                    }
                    //Writing out 
                    switch (data_mode_update)
                    {
                        case 0://HAND_LX_ACC_NO_AHRS
                            WX_LX = String.Format("{0:000.0000}", accWX_LX).ToString();
                            WY_LX = String.Format("{0:000.0000}", accWY_LX).ToString();
                            WZ_LX = String.Format("{0:000.0000}", accWZ_LX).ToString();

                            TX_LX = String.Format("{0:000.0000}", accTX_LX).ToString();
                            TY_LX = String.Format("{0:000.0000}", accTY_LX).ToString();
                            TZ_LX = String.Format("{0:000.0000}", accTZ_LX).ToString();

                            IX_LX = String.Format("{0:000.0000}", accIX_LX).ToString();
                            IY_LX = String.Format("{0:000.0000}", accIY_LX).ToString();
                            IZ_LX = String.Format("{0:000.0000}", accIZ_LX).ToString();

                            MX_LX = String.Format("{0:000.0000}", accMX_LX).ToString();
                            MY_LX = String.Format("{0:000.0000}", accMY_LX).ToString();
                            MZ_LX = String.Format("{0:000.0000}", accMZ_LX).ToString();

                            LOSS_HLX = countMicroLossRG.ToString();
                            printHandData("L");
                            break;

                        case 1://HAND_LX_GYR_NO_AHRS
                            WX_LX = String.Format("{0:000.0000}", gyrWX_LX).ToString();
                            WY_LX = String.Format("{0:000.0000}", gyrWY_LX).ToString();
                            WZ_LX = String.Format("{0:000.0000}", gyrWZ_LX).ToString();

                            TX_LX = String.Format("{0:000.0000}", gyrMX_LX).ToString();
                            TY_LX = String.Format("{0:000.0000}", gyrMY_LX).ToString();
                            TZ_LX = String.Format("{0:000.0000}", gyrMZ_LX).ToString();

                            IX_LX = String.Format("{0:000.0000}", gyrIX_LX).ToString();
                            IY_LX = String.Format("{0:000.0000}", gyrIY_LX).ToString();
                            IZ_LX = String.Format("{0:000.0000}", gyrIZ_LX).ToString();

                            MX_LX = String.Format("{0:000.0000}", gyrTX_LX).ToString();
                            MY_LX = String.Format("{0:000.0000}", gyrTY_LX).ToString();
                            MZ_LX = String.Format("{0:000.0000}", gyrTZ_LX).ToString();

                            LOSS_HLX = countMicroLossLG.ToString();
                            printHandData("L");
                            break;

                        case 2://HAND_LX_MAGN_NO_AHRS
                            WX_LX = String.Format("{0:000.0000}", magWX_LX).ToString();
                            WY_LX = String.Format("{0:000.0000}", magWY_LX).ToString();
                            WZ_LX = String.Format("{0:000.0000}", magWZ_LX).ToString();

                            TX_LX = String.Format("{0:000.0000}", magMX_LX).ToString();
                            TY_LX = String.Format("{0:000.0000}", magMY_LX).ToString();
                            TZ_LX = String.Format("{0:000.0000}", magMZ_LX).ToString();

                            IX_LX = String.Format("{0:000.0000}", magIX_LX).ToString();
                            IY_LX = String.Format("{0:000.0000}", magIY_LX).ToString();
                            IZ_LX = String.Format("{0:000.0000}", magIZ_LX).ToString();

                            MX_LX = String.Format("{0:000.0000}", magTX_LX).ToString();
                            MY_LX = String.Format("{0:000.0000}", magTY_LX).ToString();
                            MZ_LX = String.Format("{0:000.0000}", magTZ_LX).ToString();

                            LOSS_HLX = countMicroLossLG.ToString();
                            printHandData("L");
                            break;

                        case 3://HAND_LX_AHRS
                            ROLLW_LX = String.Format("{0:000.0000}", rollW_LX).ToString();
                            ROLLT_LX = String.Format("{0:000.0000}", rollM_LX).ToString();
                            ROLLI_LX = String.Format("{0:000.0000}", rollI_LX).ToString();
                            ROLLM_LX = String.Format("{0:000.0000}", rollT_LX).ToString();

                            PITCHW_LX = String.Format("{0:000.0000}", pitchW_LX).ToString();
                            PITCHT_LX = String.Format("{0:000.0000}", pitchM_LX).ToString();
                            PITCHI_LX = String.Format("{0:000.0000}", pitchI_LX).ToString();
                            PITCHM_LX = String.Format("{0:000.0000}", pitchT_LX).ToString();

                            YAWW_LX = String.Format("{0:000.0000}", yawW_LX).ToString();
                            YAWT_LX = String.Format("{0:000.0000}", yawM_LX).ToString();
                            YAWI_LX = String.Format("{0:000.0000}", yawI_LX).ToString();
                            YAWM_LX = String.Format("{0:000.0000}", yawT_LX).ToString();

                            LOSS_HLX_AHRS = countMicroLossLG.ToString();

                            printHandDataAHRS("L");

                            break;

                        case 4: //FOOT_LX_ACC_NO_AHRS

                            FX_LX = String.Format("{0:000.0000}", accxF_LX).ToString();
                            FY_LX = String.Format("{0:000.0000}", accyF_LX).ToString();
                            FZ_LX = String.Format("{0:000.0000}", acczF_LX).ToString();

                            LOSS_FLX = countMicroLossLF.ToString();
                            printFootData("L");
                            break;

                        case 5://FOOT_LX_GYR_NO_AHRS
                            FX_LX = String.Format("{0:000.0000}", gyrxF_LX).ToString();
                            FY_LX = String.Format("{0:000.0000}", gyryF_LX).ToString();
                            FZ_LX = String.Format("{0:000.0000}", gyrzF_LX).ToString();

                            LOSS_FLX = countMicroLossLF.ToString();
                            printFootData("L");
                            break;

                        case 6: //FOOT_LX_MAGN_NO_AHRS
                            FX_LX = String.Format("{0:000.0000}", magxF_LX).ToString();
                            FY_LX = String.Format("{0:000.0000}", magyF_LX).ToString();
                            FZ_LX = String.Format("{0:000.0000}", magzF_LX).ToString();

                            LOSS_FLX = countMicroLossLF.ToString();
                            printFootData("L");
                            break;

                        case 7://FOOT_LX_AHRS
                            ROLLF_LX = String.Format("{0:000.0000}", rollF_LX).ToString();
                            PITCHF_LX = String.Format("{0:000.0000}", pitchF_LX).ToString();
                            YAWF_LX = String.Format("{0:000.0000}", yawF_LX).ToString();

                            LOSS_FLX_AHRS = countMicroLossLF.ToString();
                            printFootDataAHRS("L");
                            break;

                        case 8://HAND_RX_ACC_NO_AHRS
                            WX_RX = String.Format("{0:000.0000}", accWX_RX).ToString();
                            WY_RX = String.Format("{0:000.0000}", accWY_RX).ToString();
                            WZ_RX = String.Format("{0:000.0000}", accWZ_RX).ToString();

                            TX_RX = String.Format("{0:000.0000}", accTX_RX).ToString();
                            TY_RX = String.Format("{0:000.0000}", accTY_RX).ToString();
                            TZ_RX = String.Format("{0:000.0000}", accTZ_RX).ToString();

                            IX_RX = String.Format("{0:000.0000}", accIX_RX).ToString();
                            IY_RX = String.Format("{0:000.0000}", accIY_RX).ToString();
                            IZ_RX = String.Format("{0:000.0000}", accIZ_RX).ToString();

                            MX_RX = String.Format("{0:000.0000}", accMX_RX).ToString();
                            MY_RX = String.Format("{0:000.0000}", accMY_RX).ToString();
                            MZ_RX = String.Format("{0:000.0000}", accMZ_RX).ToString();

                            LOSS_HRX = countMicroLossRG.ToString();
                            printHandData("R");
                            break;

                        case 9://HAND_RX_GYR_NO_AHRS
                            WX_RX = String.Format("{0:000.0000}", gyrWX_RX).ToString();
                            WY_RX = String.Format("{0:000.0000}", gyrWY_RX).ToString();
                            WZ_RX = String.Format("{0:000.0000}", gyrWZ_RX).ToString();

                            TX_RX = String.Format("{0:000.0000}", gyrTX_RX).ToString();
                            TY_RX = String.Format("{0:000.0000}", gyrTY_RX).ToString();
                            TZ_RX = String.Format("{0:000.0000}", gyrTZ_RX).ToString();

                            IX_RX = String.Format("{0:000.0000}", gyrIX_RX).ToString();
                            IY_RX = String.Format("{0:000.0000}", gyrIY_RX).ToString();
                            IZ_RX = String.Format("{0:000.0000}", gyrIZ_RX).ToString();

                            MX_RX = String.Format("{0:000.0000}", gyrMX_RX).ToString();
                            MY_RX = String.Format("{0:000.0000}", gyrMY_RX).ToString();
                            MZ_RX = String.Format("{0:000.0000}", gyrMZ_RX).ToString();

                            LOSS_HRX = countMicroLossRG.ToString();
                            printHandData("R");
                            break;

                        case 10://HAND_RX_MAGN_NO_AHRS
                            WX_RX = String.Format("{0:000.0000}", magWX_RX).ToString();
                            WY_RX = String.Format("{0:000.0000}", magWY_RX).ToString();
                            WZ_RX = String.Format("{0:000.0000}", magWZ_RX).ToString();

                            TX_RX = String.Format("{0:000.0000}", magTX_RX).ToString();
                            TY_RX = String.Format("{0:000.0000}", magTY_RX).ToString();
                            TZ_RX = String.Format("{0:000.0000}", magTZ_RX).ToString();

                            IX_RX = String.Format("{0:000.0000}", magIX_RX).ToString();
                            IY_RX = String.Format("{0:000.0000}", magIY_RX).ToString();
                            IZ_RX = String.Format("{0:000.0000}", magIZ_RX).ToString();

                            MX_RX = String.Format("{0:000.0000}", magMX_RX).ToString();
                            MY_RX = String.Format("{0:000.0000}", magMY_RX).ToString();
                            MZ_RX = String.Format("{0:000.0000}", magMZ_RX).ToString();

                            LOSS_HRX = countMicroLossRG.ToString();
                            printHandData("R");
                            break;

                        case 11://HAND_RX_AHRS
                            ROLLW_RX = String.Format("{0:000.0000}", rollW_RX).ToString();
                            ROLLT_RX = String.Format("{0:000.0000}", rollT_RX).ToString();
                            ROLLI_RX = String.Format("{0:000.0000}", rollI_RX).ToString();
                            ROLLM_RX = String.Format("{0:000.0000}", rollM_RX).ToString();

                            PITCHW_RX = String.Format("{0:000.0000}", pitchW_RX).ToString();
                            PITCHT_RX = String.Format("{0:000.0000}", pitchT_RX).ToString();
                            PITCHI_RX = String.Format("{0:000.0000}", pitchI_RX).ToString();
                            PITCHM_RX = String.Format("{0:000.0000}", pitchM_RX).ToString();

                            YAWW_RX = String.Format("{0:000.0000}", yawW_RX).ToString();
                            YAWT_RX = String.Format("{0:000.0000}", yawT_RX).ToString();
                            YAWI_RX = String.Format("{0:000.0000}", yawI_RX).ToString();
                            YAWM_RX = String.Format("{0:000.0000}", yawM_RX).ToString();

                            LOSS_HRX_AHRS = countMicroLossRG.ToString();
                            printHandDataAHRS("R");
                            break;

                        case 12: //FOOT_RX_ACC_NO_AHRS
                            FX_RX = String.Format("{0:000.0000}", accxF_RX).ToString();
                            FY_RX = String.Format("{0:000.0000}", accyF_RX).ToString();
                            FZ_RX = String.Format("{0:000.0000}", acczF_RX).ToString();

                            LOSS_FRX = countMicroLossRF.ToString();
                            printFootData("R");
                            break;

                        case 13://FOOT_RX_GYR_NO_AHRS
                            FX_RX = String.Format("{0:000.0000}", gyrxF_RX).ToString();
                            FY_RX = String.Format("{0:000.0000}", gyryF_RX).ToString();
                            FZ_RX = String.Format("{0:000.0000}", gyrzF_RX).ToString();

                            LOSS_FRX = countMicroLossRF.ToString();
                            printFootData("R");
                            break;

                        case 14: //FOOT_RX_MAGN_NO_AHRS
                            FX_RX = String.Format("{0:000.0000}", magxF_RX).ToString();
                            FY_RX = String.Format("{0:000.0000}", magyF_RX).ToString();
                            FZ_RX = String.Format("{0:000.0000}", magzF_RX).ToString();

                            LOSS_FRX = countMicroLossRF.ToString();
                            printFootData("R");
                            break;

                        case 15://FOOT_RX_AHRS
                            ROLLF_RX = String.Format("{0:000.0000}", rollF_RX).ToString();
                            PITCHF_RX = String.Format("{0:000.0000}", pitchF_RX).ToString();
                            YAWF_RX = String.Format("{0:000.0000}", yawF_RX).ToString();

                            LOSS_FRX_AHRS = countMicroLossRF.ToString();
                            printFootDataAHRS("R");
                            break;
                    }
                }
            }

            if ((DISCOVERY) && (count_timeout_discovery == 3))
            {
                DISCOVERY = false;
                if (IDs_FIND)
                {
                    if (!Directory.Exists(@"c:\HANDi\ID_LIST\")) { System.IO.Directory.CreateDirectory(@"c:\HANDi\ID_LIST\").ToString(); }
                    pathID = System.IO.Path.Combine(@"c:\HANDi\ID_LIST\", "DEFAULT_ID_LIST.txt");
                    if (!File.Exists(pathID))
                    {
                        StreamWriter Nome_fileID = new StreamWriter(pathID);
                        Nome_fileID.Close();
                        StreamWriter fileDatiID = File.AppendText(pathID);
                        fileDatiID.Flush();
                        fileDatiID.WriteLine(ID_FOOT_LX);
                        fileDatiID.WriteLine(ID_GLOVE_LX);
                        fileDatiID.WriteLine(ID_FOOT_RX);
                        fileDatiID.WriteLine(ID_GLOVE_RX);
                        fileDatiID.Flush();
                        fileDatiID.Close();
                        Console.WriteLine(ID_discovered_list);
                    }
                    else Console.WriteLine("A default ID list is already stored!");
                }
                else Console.WriteLine("IDs not discovered!");
            }
            if (hand_lx_connecting)
            {
                hand_lx_connecting = false;
                connected = true;
            }
            else if (hand_lx_disconnecting)
            {
                hand_lx_disconnecting = false; connected = false;
            }
            if (hand_rx_connecting)
            {
                hand_rx_connecting = false;
                connected = true;
            }
            else if (hand_rx_disconnecting)
            {
                hand_rx_disconnecting = false; connected = false;
            }
            if (foot_lx_connecting)
            {
                foot_lx_connecting = false; connected = true;
            }
            else if (foot_lx_disconnecting)
            {
                foot_lx_disconnecting = false; connected = false;
            }
            if (foot_rx_connecting)
            {
                foot_rx_connecting = false; connected = true;
            }
            else if (foot_rx_disconnecting)
            {
                foot_rx_disconnecting = false; connected = false;
            }
        }


        //-------------------------------------------------------------------------------
        //Read data from COM 
        public void COMRead()
        {
            while (!rec)//NO DATA TRANSMISSION
            {
                while (DISCOVERY)
                {
                    if (!IDs_FIND)
                    {
                        try
                        {
                            if (SP1.BytesToRead > 0) count_timeout_discovery = 0;
                            string ID = SP1.ReadLine();
                            if (ID.Contains("LEFT_GLOVE"))
                            { ID_GLOVE_LX = ID.Substring(13, 12); }
                            else if (ID.Contains("LEFT_FOOT"))
                            { ID_FOOT_LX = ID.Substring(13, 12); }
                            else if (ID.Contains("RIGHT_GLOVE"))
                            { ID_GLOVE_RX = ID.Substring(13, 12); }
                            else if (ID.Contains("RIGHT_FOOT"))
                            { ID_FOOT_RX = ID.Substring(13, 12); }
                        }
                        catch (TimeoutException)
                        {
                            count_timeout_discovery++;
                            if (count_timeout_discovery == 3)
                            {
                                if ((ID_GLOVE_LX != null) && (ID_FOOT_LX != null) && (ID_FOOT_RX != null) && (ID_GLOVE_RX != null))
                                {
                                    IDs_FIND = true;
                                    ID_discovered_list = "LEFT FOOT ID: " + ID_FOOT_LX + "\r\n"
                                                       + "LEFT GLOVE ID: " + ID_GLOVE_LX + "\r\n"
                                                       + "RIGHT FOOT ID: " + ID_FOOT_RX + "\r\n"
                                                       + "RIGHT GLOVE ID: " + ID_GLOVE_RX;
                                }
                                else
                                { IDs_FIND = false; }
                                UpdatePanel(workMode);
                            }
                        }
                    }
                    else
                    {
                        count_timeout_discovery = 3;
                        ID_discovered_list = "LEFT FOOT ID: " + ID_FOOT_LX + "\r\n"
                                           + "LEFT GLOVE ID: " + ID_GLOVE_LX + "\r\n"
                                           + "RIGHT FOOT ID: " + ID_FOOT_RX + "\r\n"
                                           + "RIGHT GLOVE ID: " + ID_GLOVE_RX;
                        UpdatePanel(workMode);

                    }
                }
                while (hand_lx_connecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("ConnectionUp " + ID_GLOVE_LX)) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("BypassMode"))
                            {
                                ID_READ = SP1.ReadLine();
                                if (ID_READ.Contains("0006CUPL"))
                                {
                                    FOOT_NODE_LX = '1'; GLOVE_NODE_LX = '0'; GLOVE_NODE_RX = '1';
                                    CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                    if ((!foot_lx_waiting_for_connection) && (!hand_rx_waiting_for_connection))
                                    {
                                        UpdatePanel(workMode);
                                    }
                                    else
                                    {
                                        if (foot_lx_waiting_for_connection) { foot_lx_waiting_for_connection = false; foot_lx_connecting = true; }
                                        else if (hand_rx_waiting_for_connection) { hand_rx_waiting_for_connection = false; hand_rx_connecting = true; }
                                        hand_lx_connecting = false;
                                        CONNECTION_STEP1 = true;
                                        Thread.Sleep(100);
                                        SP1.Write("^#^$^%");
                                    }
                                }
                                else if (ID_READ.Contains("1006CUPL"))
                                {
                                    FOOT_NODE_LX = '0'; GLOVE_NODE_LX = '1'; GLOVE_NODE_RX = '0';
                                    CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                    if ((!foot_lx_waiting_for_connection) && (!hand_rx_waiting_for_connection))
                                    {
                                        UpdatePanel(workMode);
                                    }
                                    else
                                    {
                                        if (foot_lx_waiting_for_connection) { foot_lx_waiting_for_connection = false; foot_lx_connecting = true; }
                                        else if (hand_rx_waiting_for_connection) { hand_rx_waiting_for_connection = false; hand_rx_connecting = true; }
                                        hand_lx_connecting = false;
                                        CONNECTION_STEP1 = true;
                                        Thread.Sleep(100);
                                        SP1.Write("^#^$^%");
                                    }
                                }
                            }
                            else { }
                        }
                        else if ((!ID_READ.Contains("ConnectionUp " + ID_GLOVE_LX)) && (CONNECTION_STEP2))
                        {
                            SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");
                        }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            Thread.Sleep(100); CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }
                    }
                    catch
                    {
                        hand_lx_connecting_count++;
                        try
                        {
                            if (ID_READ.Contains("CommandMode"))
                            {
                                hand_lx_connecting_count = 0;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_LX + "\r\n");
                            }
                            else if (ID_READ.Contains("0006CUPL"))
                            {
                                FOOT_NODE_LX = '1'; GLOVE_NODE_LX = '0'; GLOVE_NODE_RX = '1'; hand_lx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if ((!foot_lx_waiting_for_connection) && (!hand_rx_waiting_for_connection))
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    if (foot_lx_waiting_for_connection) { foot_lx_waiting_for_connection = false; foot_lx_connecting = true; }
                                    else if (hand_rx_waiting_for_connection) { hand_rx_waiting_for_connection = false; hand_rx_connecting = true; }
                                    hand_lx_connecting = false;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                            else if (ID_READ.Contains("1006CUPL"))
                            {
                                FOOT_NODE_LX = '0'; GLOVE_NODE_LX = '1'; GLOVE_NODE_RX = '0'; hand_lx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if ((!foot_lx_waiting_for_connection) && (!hand_rx_waiting_for_connection))
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    if (foot_lx_waiting_for_connection) { foot_lx_waiting_for_connection = false; foot_lx_connecting = true; }
                                    else if (hand_rx_waiting_for_connection) { hand_rx_waiting_for_connection = false; hand_rx_connecting = true; }
                                    hand_lx_connecting = false;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                        }
                        catch (NullReferenceException) { }
                    }
                }

                while (hand_rx_connecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("ConnectionUp " + ID_GLOVE_RX)) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("BypassMode"))
                            {
                                ID_READ = SP1.ReadLine();
                                if (ID_READ.Contains("0006CUPR"))
                                {
                                    FOOT_NODE_RX = '1'; GLOVE_NODE_RX = '0'; GLOVE_NODE_LX = '1';
                                    CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                    if (!foot_rx_waiting_for_connection)
                                    {
                                        UpdatePanel(workMode);
                                    }
                                    else
                                    {
                                        foot_rx_waiting_for_connection = false;
                                        hand_rx_connecting = false;
                                        foot_rx_connecting = true;
                                        CONNECTION_STEP1 = true;
                                        Thread.Sleep(100);
                                        SP1.Write("^#^$^%");
                                    }
                                }
                                else if (ID_READ.Contains("1006CUPR"))
                                {
                                    FOOT_NODE_RX = '0'; GLOVE_NODE_RX = '1'; GLOVE_NODE_LX = '0';
                                    CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                    if (!foot_rx_waiting_for_connection)
                                    {
                                        UpdatePanel(workMode);
                                    }
                                    else
                                    {
                                        foot_rx_waiting_for_connection = false;
                                        hand_rx_connecting = false;
                                        foot_rx_connecting = true;
                                        CONNECTION_STEP1 = true;
                                        Thread.Sleep(100);
                                        SP1.Write("^#^$^%");
                                    }
                                }
                            }
                            else { }
                        }
                        else if ((!ID_READ.Contains("ConnectionUp " + ID_GLOVE_RX)) && (CONNECTION_STEP2))
                        {
                            SP1.Write("AT+AB SPPConnect " + ID_GLOVE_RX + "\r\n");
                        }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            Thread.Sleep(100); CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPConnect " + ID_GLOVE_RX + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }
                    }
                    catch
                    {
                        hand_rx_connecting_count++;
                        try
                        {
                            if (ID_READ.Contains("CommandMode"))
                            {
                                hand_rx_connecting_count = 0;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_GLOVE_RX + "\r\n");
                            }
                            else if (ID_READ.Contains("0006CUPR"))
                            {
                                FOOT_NODE_RX = '1'; GLOVE_NODE_RX = '0'; GLOVE_NODE_LX = '1'; hand_rx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if (!foot_rx_waiting_for_connection)
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    foot_rx_waiting_for_connection = false;
                                    hand_rx_connecting = false;
                                    foot_rx_connecting = true;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                            else if (ID_READ.Contains("1006CUPR"))
                            {
                                FOOT_NODE_RX = '0'; GLOVE_NODE_RX = '1'; GLOVE_NODE_LX = '0'; hand_rx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if (!foot_rx_waiting_for_connection)
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    foot_rx_waiting_for_connection = false;
                                    hand_rx_connecting = false;
                                    foot_rx_connecting = true;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                        }
                        catch (NullReferenceException) { }
                    }
                }

                while (foot_lx_connecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("ConnectionUp " + ID_FOOT_LX)) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            try
                            {
                                Thread.Sleep(500);
                                ID_READ = SP1.ReadLine();
                                if (ID_READ.Contains("BypassMode"))
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("0006CUPL"))
                                    {
                                        FOOT_NODE_LX = '0'; GLOVE_NODE_LX = '1'; FOOT_NODE_RX = '1';
                                        CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                        if (!foot_rx_waiting_for_connection)
                                        {
                                            UpdatePanel(workMode);
                                        }
                                        else
                                        {
                                            foot_rx_waiting_for_connection = false;
                                            foot_lx_connecting = false;
                                            foot_rx_connecting = true;
                                            CONNECTION_STEP1 = true;
                                            Thread.Sleep(100);
                                            SP1.Write("^#^$^%");
                                        }
                                    }
                                    else if (ID_READ.Contains("1006CUPL"))
                                    {
                                        FOOT_NODE_LX = '1'; GLOVE_NODE_LX = '0'; FOOT_NODE_RX = '0';
                                        CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                        if (!foot_rx_waiting_for_connection)
                                        {
                                            UpdatePanel(workMode);
                                        }
                                        else
                                        {
                                            foot_rx_waiting_for_connection = false;
                                            foot_lx_connecting = false;
                                            foot_rx_connecting = true;
                                            CONNECTION_STEP1 = true;
                                            Thread.Sleep(100);
                                            SP1.Write("^#^$^%");
                                        }
                                    }
                                }
                                else { }
                            }
                            catch { }
                        }
                        else if ((!ID_READ.Contains("ConnectionUp " + ID_FOOT_LX)) && (CONNECTION_STEP2))
                        {
                            SP1.Write("AT+AB SPPConnect " + ID_FOOT_LX + "\r\n");
                        }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            Thread.Sleep(100); CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPConnect " + ID_FOOT_LX + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }
                    }
                    catch
                    {
                        foot_lx_connecting_count++;
                        try
                        {
                            if (ID_READ.Contains("CommandMode"))
                            {
                                foot_lx_connecting_count = 0;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_FOOT_LX + "\r\n");
                            }
                            else if (ID_READ.Contains("0006CUPL"))
                            {
                                FOOT_NODE_LX = '0'; GLOVE_NODE_LX = '1'; FOOT_NODE_RX = '1'; foot_lx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if (!foot_rx_waiting_for_connection)
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    foot_rx_waiting_for_connection = false;
                                    foot_lx_connecting = false;
                                    foot_rx_connecting = true;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                            else if (ID_READ.Contains("1006CUPL"))
                            {
                                FOOT_NODE_LX = '1'; GLOVE_NODE_LX = '0'; FOOT_NODE_RX = '0'; foot_lx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                if (!foot_rx_waiting_for_connection)
                                {
                                    UpdatePanel(workMode);
                                }
                                else
                                {
                                    foot_rx_waiting_for_connection = false;
                                    foot_lx_connecting = false;
                                    foot_rx_connecting = true;
                                    CONNECTION_STEP1 = true;
                                    Thread.Sleep(100);
                                    SP1.Write("^#^$^%");
                                }
                            }
                        }
                        catch (NullReferenceException) { }
                    }
                }

                while (foot_rx_connecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("ConnectionUp " + ID_FOOT_RX)) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            try
                            {
                                Thread.Sleep(500);
                                ID_READ = SP1.ReadLine();
                                if (ID_READ.Contains("BypassMode"))
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("0006CUPR"))
                                    {
                                        FOOT_NODE_RX = '0'; GLOVE_NODE_RX = '1'; FOOT_NODE_LX = '1';
                                        CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                        UpdatePanel(workMode);
                                    }
                                    else if (ID_READ.Contains("1006CUPR"))
                                    {
                                        FOOT_NODE_RX = '1'; GLOVE_NODE_RX = '0'; FOOT_NODE_LX = '0';
                                        CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                        UpdatePanel(workMode);
                                    }
                                }
                                else { }
                            }
                            catch { }
                        }
                        else if ((!ID_READ.Contains("ConnectionUp " + ID_FOOT_RX)) && (CONNECTION_STEP2))
                        {
                            SP1.Write("AT+AB SPPConnect " + ID_FOOT_RX + "\r\n");
                        }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            Thread.Sleep(100); CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPConnect " + ID_FOOT_RX + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }
                    }
                    catch
                    {
                        foot_rx_connecting_count++;
                        try
                        {
                            if (ID_READ.Contains("CommandMode"))
                            {
                                foot_rx_connecting_count = 0;
                                CONNECTION_STEP2 = true;
                                SP1.Write("AT+AB SPPConnect " + ID_FOOT_RX + "\r\n");
                            }
                            else if (ID_READ.Contains("0006CUPR"))
                            {
                                FOOT_NODE_RX = '0'; GLOVE_NODE_RX = '1'; FOOT_NODE_LX = '1'; foot_rx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                UpdatePanel(workMode);
                            }
                            else if (ID_READ.Contains("1006CUPR"))
                            {
                                FOOT_NODE_RX = '1'; GLOVE_NODE_RX = '0'; FOOT_NODE_LX = '0'; foot_rx_connecting_count = 0; CONNECTION_STEP1 = false; CONNECTION_STEP2 = false;
                                UpdatePanel(workMode);
                            }
                        }
                        catch (NullReferenceException) { }
                    }
                }

                while (hand_lx_disconnecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        {
                            CONNECTION_STEP3 = false;
                            if ((!foot_lx_waiting_for_disconnection) && (!hand_rx_waiting_for_disconnection)) UpdatePanel(workMode);//this.Invoke(new UpdateGUI(this.UpdatePanel));
                            else
                            {
                                if (foot_lx_waiting_for_disconnection) { foot_lx_waiting_for_disconnection = false; foot_lx_disconnecting = true; }
                                else if (hand_rx_waiting_for_disconnection) { hand_rx_waiting_for_disconnection = false; hand_rx_disconnecting = true; }
                                hand_lx_disconnecting = false;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                        else if ((!ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        { SP1.Write("AT+AB Bypass\r\n"); }
                        if ((ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("AT-AB SPPConnectionClosed"))
                            {
                                try
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("AT-AB ConnectionDown"))
                                    {

                                        if ((!foot_lx_waiting_for_disconnection) && (!hand_rx_waiting_for_disconnection)) UpdatePanel(workMode); //this.Invoke(new UpdateGUI(this.UpdatePanel));
                                        else
                                        {
                                            if (foot_lx_waiting_for_disconnection) { foot_lx_waiting_for_disconnection = false; foot_lx_disconnecting = true; }
                                            else if (hand_rx_waiting_for_disconnection) { hand_rx_waiting_for_disconnection = false; hand_rx_disconnecting = true; }
                                            hand_lx_disconnecting = false;
                                            CONNECTION_STEP1 = true;
                                            Thread.Sleep(100);
                                            SP1.Write("^#^$^%");
                                        }
                                    }
                                }
                                catch (Exception) { SP1.Write("AT+AB Bypass\r\n"); CONNECTION_STEP3 = true; }
                            }
                        }
                        else if ((!ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        { SP1.Write("AT+AB SPPDisconnect " + GLOVE_NODE_LX.ToString() + "\r\n"); }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPDisconnect " + GLOVE_NODE_LX.ToString() + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }

                    }
                    catch
                    {
                        hand_lx_disconnecting_count++;
                        if (hand_lx_disconnecting_count == 3)
                        {
                            hand_lx_disconnecting_count = 0;
                            CONNECTION_STEP3 = false; CONNECTION_STEP2 = false; CONNECTION_STEP1 = false;
                            if ((!foot_lx_waiting_for_disconnection) && (!hand_rx_waiting_for_disconnection))
                            {
                                try
                                {
                                    UpdatePanel(workMode);
                                }
                                catch { }
                            }
                            else
                            {
                                if (foot_lx_waiting_for_disconnection) { foot_lx_waiting_for_disconnection = false; foot_lx_disconnecting = true; }
                                else if (hand_rx_waiting_for_disconnection) { hand_rx_waiting_for_disconnection = false; hand_rx_disconnecting = true; }
                                hand_lx_disconnecting = false;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                    }
                }

                while (hand_rx_disconnecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        {
                            CONNECTION_STEP3 = false;
                            if (!foot_rx_waiting_for_disconnection) UpdatePanel(workMode);
                            else
                            {
                                foot_rx_waiting_for_disconnection = false;
                                foot_rx_disconnecting = true;
                                hand_rx_disconnecting = false;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                        else if ((!ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        { SP1.Write("AT+AB Bypass\r\n"); }
                        if ((ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("AT-AB SPPConnectionClosed"))
                            {
                                try
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("AT-AB ConnectionDown"))
                                    {
                                        if (!foot_rx_waiting_for_disconnection) UpdatePanel(workMode);                                        else
                                        {
                                            foot_rx_waiting_for_disconnection = false;
                                            hand_rx_disconnecting = false;
                                            foot_rx_disconnecting = true;
                                            CONNECTION_STEP1 = true;
                                            Thread.Sleep(100);
                                            SP1.Write("^#^$^%");
                                        }
                                    }
                                }
                                catch (Exception) { SP1.Write("AT+AB Bypass\r\n"); CONNECTION_STEP3 = true; }
                            }
                        }
                        else if ((!ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        { SP1.Write("AT+AB SPPDisconnect " + GLOVE_NODE_RX.ToString() + "\r\n"); }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPDisconnect " + GLOVE_NODE_RX.ToString() + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }

                    }
                    catch
                    {
                        hand_rx_disconnecting_count++;
                        if (hand_rx_disconnecting_count == 3)
                        {
                            hand_rx_disconnecting_count = 0;
                            CONNECTION_STEP3 = false; CONNECTION_STEP2 = false; CONNECTION_STEP1 = false;
                            if (!foot_rx_waiting_for_disconnection)
                            {
                                try
                                {
                                    UpdatePanel(workMode);
                                }
                                catch { }
                            }
                            else
                            {
                                foot_rx_waiting_for_disconnection = false;
                                hand_rx_disconnecting = false;
                                foot_rx_disconnecting = true;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                    }
                }

                while (foot_lx_disconnecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        {
                            CONNECTION_STEP3 = false;
                            if (!foot_rx_waiting_for_disconnection)
                            {
                                try
                                {
                                    UpdatePanel(workMode);
                                }
                                catch { }
                            }
                            else
                            {
                                foot_rx_waiting_for_disconnection = false;
                                foot_rx_disconnecting = true;
                                foot_lx_disconnecting = false;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                        else if ((!ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        { SP1.Write("AT+AB Bypass\r\n"); }
                        if ((ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("AT-AB SPPConnectionClosed"))
                            {
                                try
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("AT-AB ConnectionDown"))
                                    {
                                        if (!foot_rx_waiting_for_disconnection)
                                        {
                                            try
                                            {
                                                UpdatePanel(workMode);
                                            }
                                            catch { }
                                        }
                                        else
                                        {
                                            foot_rx_waiting_for_disconnection = false;
                                            foot_rx_disconnecting = true;
                                            foot_lx_disconnecting = false;
                                            CONNECTION_STEP1 = true;
                                            Thread.Sleep(100);
                                            SP1.Write("^#^$^%");
                                        }
                                    }
                                }
                                catch (Exception) { SP1.Write("AT+AB Bypass\r\n"); CONNECTION_STEP3 = true; }
                            }
                        }
                        else if ((!ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        { SP1.Write("AT+AB SPPDisconnect " + FOOT_NODE_LX.ToString() + "\r\n"); }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPDisconnect " + FOOT_NODE_LX.ToString() + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }

                    }
                    catch
                    {
                        foot_lx_disconnecting_count++;
                        if (foot_lx_disconnecting_count == 3)
                        {
                            foot_lx_disconnecting_count = 0;
                            CONNECTION_STEP3 = false; CONNECTION_STEP2 = false; CONNECTION_STEP1 = false;
                            if (!foot_rx_waiting_for_disconnection)
                            {
                                try
                                {
                                    UpdatePanel(workMode);
                                }
                                catch { }
                            }
                            else
                            {
                                foot_rx_waiting_for_disconnection = false;
                                foot_rx_disconnecting = true;
                                foot_lx_disconnecting = false;
                                CONNECTION_STEP1 = true;
                                Thread.Sleep(100);
                                SP1.Write("^#^$^%");
                            }
                        }
                    }
                }

                while (foot_rx_disconnecting)
                {
                    try
                    {
                        ID_READ = SP1.ReadLine();
                        if ((ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        {
                            CONNECTION_STEP3 = false;
                            UpdatePanel(workMode);
                        }
                        else if ((!ID_READ.Contains("BypassMode")) && (CONNECTION_STEP3))
                        { SP1.Write("AT+AB Bypass\r\n"); }
                        if ((ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        {
                            CONNECTION_STEP2 = false;
                            ID_READ = SP1.ReadLine();
                            if (ID_READ.Contains("AT-AB SPPConnectionClosed"))
                            {
                                try
                                {
                                    ID_READ = SP1.ReadLine();
                                    if (ID_READ.Contains("AT-AB ConnectionDown"))
                                    {
                                        UpdatePanel(workMode);
                                    }
                                }
                                catch (Exception) { SP1.Write("AT+AB Bypass\r\n"); CONNECTION_STEP3 = true; }
                            }
                        }
                        else if ((!ID_READ.Contains("AT-AB SPPConnectionClosed")) && (CONNECTION_STEP2))
                        { SP1.Write("AT+AB SPPDisconnect " + FOOT_NODE_RX.ToString() + "\r\n"); }
                        if ((ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        {
                            CONNECTION_STEP1 = false; CONNECTION_STEP2 = true;
                            SP1.Write("AT+AB SPPDisconnect " + FOOT_NODE_RX.ToString() + "\r\n");
                        }
                        else if ((!ID_READ.Contains("CommandMode")) && (CONNECTION_STEP1))
                        { SP1.Write("^#^$^%"); }

                    }
                    catch
                    {
                        foot_rx_disconnecting_count++;
                        if (foot_rx_disconnecting_count == 3)
                        {
                            foot_rx_disconnecting_count = 0;
                            CONNECTION_STEP3 = false; CONNECTION_STEP2 = false; CONNECTION_STEP1 = false;
                            UpdatePanel(workMode);
                        }
                    }
                }
            }
            //----------------------------------------------------------------------------------
            while (rec)//DATA TRANSMISSION
            {
                //condition for exit recording 
                if (Console.KeyAvailable) break;
                switch (working_mode)
                {
                    case 0:/*HAND_LX*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char HAND_NODE_NUM_LX = (char)SP1.ReadByte();
                                    if (HAND_NODE_NUM_LX == GLOVE_NODE_LX) GLOVE_PACKET_READ_LX();
                                    else ERROR_LG++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false; 
                                RESET_STOP = true;
                                Console.WriteLine("LX GLOVE CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 1:/*HAND_RX*/
                        try
                        {
                            try
                            {

                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char HAND_NODE_NUM_RX = (char)SP1.ReadByte();
                                    if (HAND_NODE_NUM_RX == GLOVE_NODE_RX) GLOVE_PACKET_READ_RX();
                                    else ERROR_RG++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("RX GLOVE CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 2:/*FOOT_LX*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char FOOT_NODE_NUM_LX = (char)SP1.ReadByte();
                                    if (FOOT_NODE_NUM_LX == FOOT_NODE_LX) FOOT_PACKET_READ_LX();
                                    else ERROR_LF++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("LX FOOT CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 3:/*FOOT_RX*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char FOOT_NODE_NUM_RX = (char)SP1.ReadByte();
                                    if (FOOT_NODE_NUM_RX == FOOT_NODE_RX) FOOT_PACKET_READ_RX();
                                    else ERROR_RF++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("RX FOOT CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 4:/*BOTH_HAND*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char NODE_NUM = (char)SP1.ReadByte();
                                    if (NODE_NUM == GLOVE_NODE_LX) GLOVE_PACKET_READ_LX();
                                    else if (NODE_NUM == GLOVE_NODE_RX) GLOVE_PACKET_READ_RX();
                                    else ERROR_LG++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("HAND CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 5:/*BOTH_FOOT*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char NODE_NUM = (char)SP1.ReadByte();
                                    if (NODE_NUM == FOOT_NODE_LX) FOOT_PACKET_READ_LX();
                                    else if (NODE_NUM == FOOT_NODE_RX) FOOT_PACKET_READ_RX();
                                    else ERROR_LF++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("FOOT CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 6:/*HAND_FOOT_LX*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char NODE_NUM_LX = (char)SP1.ReadByte();
                                    if (NODE_NUM_LX == GLOVE_NODE_LX) GLOVE_PACKET_READ_LX();
                                    else if (NODE_NUM_LX == FOOT_NODE_LX) FOOT_PACKET_READ_LX();
                                    else ERROR_LG++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("LX GLOVE AND FOOT CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);
                            }
                        }
                        break;
                    case 7:/*HAND_FOOT_RX*/
                        try
                        {
                            try
                            {
                                if (SP1.BytesToRead > 0)
                                {
                                    Thread.BeginCriticalRegion();
                                    char NODE_NUM_RX = (char)SP1.ReadByte();
                                    if (NODE_NUM_RX == GLOVE_NODE_RX) GLOVE_PACKET_READ_RX();
                                    else if (NODE_NUM_RX == FOOT_NODE_RX) FOOT_PACKET_READ_RX();
                                    else ERROR_RG++;
                                    Thread.EndCriticalRegion();
                                }
                                else { }
                            }
                            catch (IOException)
                            { }
                        }
                        catch (TimeoutException)
                        {
                            if (!RESET_STOP)
                            {
                                rec = false;
                                RESET_STOP = true;
                                Console.WriteLine("RX GLOVE AND FOOT CONNECTION LOST! SYSTEM ERROR, THE APPLICATION WILL BE RESTART AUTOMATICALLY!"); RESET = true;
                                UpdatePanel(workMode);

                            }
                        }
                        break;
                }
            }
            AppClose();
        }


        /*---------------------------------------------------------------------------------------*/
        public void saveNetworkID()
        {
            if (COM_OPEN)
            {
                Console.WriteLine("Device ID discovery. Wait until ID list storage."); DISCOVERY = true;
                SP1.Write("AT+AB Discovery\r\n");
            }
            else { Console.WriteLine("WARNING: COM CLOSE OR NOT FOUND!"); }

        }
        
        /*---------------------------------------------------------------------------------------*/
        public void deleteNetworkID()
        {
            if (File.Exists(@"c:\HANDi\ID_LIST\DEFAULT_ID_LIST.txt"))
            {
                File.Delete(@"c:\HANDi\ID_LIST\DEFAULT_ID_LIST.txt");
            }
        }
        
        /*---------------------------------------------------------------------------------------*/
        void GLOVE_PACKET_READ_LX()
        {
            char[] LENGTH_CHAR_GL = new char[3];
            LENGTH_CHAR_GL[0] = (char)SP1.ReadByte();
            for (int i = 1; i < 3; i++) LENGTH_CHAR_GL[i] = (char)SP1.ReadByte();
            string LENGTH_GL = string.Concat(LENGTH_CHAR_GL);
            int LENGTH_INT_GL = 0;
            try
            {
                LENGTH_INT_GL = int.Parse(LENGTH_GL);
                if (INDEX_GL == 0)
                {
                    GLOVE_PACKET_LX[0] = (byte)SP1.ReadByte();
                    if ((char)GLOVE_PACKET_LX[0] == 'G')
                    {
                        if (LENGTH_INT_GL >= PACKET_NUM_G)
                        {
                            for (int i = 1; i < 3; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            if (((char)GLOVE_PACKET_LX[1] == 'L') && ((char)GLOVE_PACKET_LX[2] == 'T'))
                            {
                                INDEX_GL = INDEX_GL + 3;
                                while (LENGTH_INT_GL >= PACKET_NUM_G)
                                {
                                    for (int i = INDEX_GL; i < PACKET_NUM_G; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                                    PACKET_ASSESSMENT_LG();
                                    if (PACKET_GL_COMPLETED)
                                    {
                                        LENGTH_INT_GL = LENGTH_INT_GL - PACKET_NUM_G;
                                        INDEX_GL = 0; PACKET_GL_COMPLETED = false;
                                        if ((LENGTH_INT_GL > 0) && (LENGTH_INT_GL < PACKET_NUM_G))
                                        {
                                            GLOVE_PACKET_LX[0] = (byte)SP1.ReadByte();
                                            if ((char)GLOVE_PACKET_LX[0] == 'G')
                                            {
                                                INDEX_GL++;
                                                for (int i = INDEX_GL; i < LENGTH_INT_GL; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                                                INDEX_GL = LENGTH_INT_GL; PACKET_GL_COMPLETED = false;
                                            }
                                            else { ERROR_LG1++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                                        }
                                    }
                                    else { ERROR_LG2++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                                }
                            }
                            else { ERROR_LG3++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                        }
                        else if (LENGTH_INT_GL < PACKET_NUM_G)
                        {
                            INDEX_GL++;
                            for (int i = INDEX_GL; i < LENGTH_INT_GL; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            INDEX_GL = LENGTH_INT_GL; PACKET_GL_COMPLETED = false;
                        }
                    }
                    else { ERROR_LG4++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                }
                else if (INDEX_GL > 0)
                {
                    if (LENGTH_INT_GL >= PACKET_NUM_G)
                    {
                        while (LENGTH_INT_GL >= PACKET_NUM_G)
                        {
                            for (int i = INDEX_GL; i < PACKET_NUM_G; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LG();
                            if (PACKET_GL_COMPLETED)
                            {
                                LENGTH_INT_GL = LENGTH_INT_GL - (PACKET_NUM_G - INDEX_GL);
                                INDEX_GL = 0; PACKET_GL_COMPLETED = false;
                                if ((LENGTH_INT_GL > 0) && (LENGTH_INT_GL < PACKET_NUM_G))
                                {
                                    GLOVE_PACKET_LX[0] = (byte)SP1.ReadByte();
                                    if ((char)GLOVE_PACKET_LX[0] == 'G')
                                    {
                                        INDEX_GL++;
                                        for (int i = INDEX_GL; i < LENGTH_INT_GL; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                                        INDEX_GL = LENGTH_INT_GL; PACKET_GL_COMPLETED = false;
                                    }
                                    else { ERROR_LG5++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                                }
                            }
                            else { ERROR_LG6++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                        }
                    }
                    else if (LENGTH_INT_GL < PACKET_NUM_G)
                    {
                        if (LENGTH_INT_GL + INDEX_GL == PACKET_NUM_G)
                        {
                            for (int i = INDEX_GL; i < PACKET_NUM_G; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LG();
                            if (PACKET_GL_COMPLETED) { INDEX_GL = 0; PACKET_GL_COMPLETED = false; }
                            else { ERROR_LG7++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                        }
                        else if (LENGTH_INT_GL + INDEX_GL < PACKET_NUM_G)
                        {
                            for (int i = INDEX_GL; i < LENGTH_INT_GL + INDEX_GL; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            INDEX_GL = LENGTH_INT_GL + INDEX_GL; PACKET_GL_COMPLETED = false;
                        }
                        else if (LENGTH_INT_GL + INDEX_GL > PACKET_NUM_G)
                        {
                            for (int i = INDEX_GL; i < PACKET_NUM_G; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LG();
                            if (PACKET_GL_COMPLETED)
                            {
                                LENGTH_INT_GL = LENGTH_INT_GL - (PACKET_NUM_G - INDEX_GL);
                                INDEX_GL = 0; PACKET_GL_COMPLETED = false;
                                for (int i = INDEX_GL; i < LENGTH_INT_GL; i++) GLOVE_PACKET_LX[i] = (byte)SP1.ReadByte();
                                INDEX_GL = LENGTH_INT_GL; PACKET_GL_COMPLETED = false;
                            }
                            else { ERROR_LG8++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
                        }
                    }
                }
            }
            catch { ERROR_LG9++; INDEX_GL = 0; LENGTH_INT_GL = 0; }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void GLOVE_PACKET_READ_RX()
        {
            char[] LENGTH_CHAR_GR = new char[3];
            LENGTH_CHAR_GR[0] = (char)SP1.ReadByte();
            for (int i = 1; i < 3; i++) LENGTH_CHAR_GR[i] = (char)SP1.ReadByte();
            string LENGTH_GR = string.Concat(LENGTH_CHAR_GR);
            int LENGTH_INT_GR = 0;
            try
            {
                LENGTH_INT_GR = int.Parse(LENGTH_GR);
                if (INDEX_GR == 0)
                {
                    GLOVE_PACKET_RX[0] = (byte)SP1.ReadByte();
                    if ((char)GLOVE_PACKET_RX[0] == 'G')
                    {
                        if (LENGTH_INT_GR >= PACKET_NUM_G)
                        {
                            for (int i = 1; i < 3; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            if (((char)GLOVE_PACKET_RX[1] == 'R') && ((char)GLOVE_PACKET_RX[2] == 'H'))
                            {
                                INDEX_GR = INDEX_GR + 3;
                                while (LENGTH_INT_GR >= PACKET_NUM_G)
                                {
                                    for (int i = INDEX_GR; i < PACKET_NUM_G; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                                    PACKET_ASSESSMENT_RG();
                                    if (PACKET_GR_COMPLETED)
                                    {
                                        LENGTH_INT_GR = LENGTH_INT_GR - PACKET_NUM_G;
                                        INDEX_GR = 0; PACKET_GR_COMPLETED = false;
                                        if ((LENGTH_INT_GR > 0) && (LENGTH_INT_GR < PACKET_NUM_G))
                                        {
                                            GLOVE_PACKET_RX[0] = (byte)SP1.ReadByte();
                                            if ((char)GLOVE_PACKET_RX[0] == 'G')
                                            {
                                                INDEX_GR++;
                                                for (int i = INDEX_GR; i < LENGTH_INT_GR; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                                                INDEX_GR = LENGTH_INT_GR; PACKET_GR_COMPLETED = false;
                                            }
                                            else { ERROR_RG1++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                                        }
                                    }
                                    else { ERROR_RG2++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                                }
                            }
                            else { ERROR_RG3++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                        }
                        else if (LENGTH_INT_GR < PACKET_NUM_G)
                        {
                            INDEX_GR++;
                            for (int i = INDEX_GR; i < LENGTH_INT_GR; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            INDEX_GR = LENGTH_INT_GR; PACKET_GR_COMPLETED = false;
                        }
                    }
                    else { ERROR_RG4++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                }
                else if (INDEX_GR > 0)
                {
                    if (LENGTH_INT_GR >= PACKET_NUM_G)
                    {
                        while (LENGTH_INT_GR >= PACKET_NUM_G)
                        {
                            for (int i = INDEX_GR; i < PACKET_NUM_G; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RG();
                            if (PACKET_GR_COMPLETED)
                            {
                                LENGTH_INT_GR = LENGTH_INT_GR - (PACKET_NUM_G - INDEX_GR);
                                INDEX_GR = 0; PACKET_GR_COMPLETED = false;
                                if ((LENGTH_INT_GR > 0) && (LENGTH_INT_GR < PACKET_NUM_G))
                                {
                                    GLOVE_PACKET_RX[0] = (byte)SP1.ReadByte();
                                    if ((char)GLOVE_PACKET_RX[0] == 'G')
                                    {
                                        INDEX_GR++;
                                        for (int i = INDEX_GR; i < LENGTH_INT_GR; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                                        INDEX_GR = LENGTH_INT_GR; PACKET_GR_COMPLETED = false;
                                    }
                                    else { ERROR_RG5++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                                }
                            }
                            else { ERROR_RG6++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                        }
                    }
                    else if (LENGTH_INT_GR < PACKET_NUM_G)
                    {
                        if (LENGTH_INT_GR + INDEX_GR == PACKET_NUM_G)
                        {
                            for (int i = INDEX_GR; i < PACKET_NUM_G; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RG();
                            if (PACKET_GR_COMPLETED) { INDEX_GR = 0; PACKET_GR_COMPLETED = false; }
                            else { ERROR_RG7++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                        }
                        else if (LENGTH_INT_GR + INDEX_GR < PACKET_NUM_G)
                        {
                            for (int i = INDEX_GR; i < LENGTH_INT_GR + INDEX_GR; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            INDEX_GR = LENGTH_INT_GR + INDEX_GR; PACKET_GR_COMPLETED = false;
                        }
                        else if (LENGTH_INT_GR + INDEX_GR > PACKET_NUM_G)
                        {
                            for (int i = INDEX_GR; i < PACKET_NUM_G; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RG();
                            if (PACKET_GR_COMPLETED)
                            {
                                LENGTH_INT_GR = LENGTH_INT_GR - (PACKET_NUM_G - INDEX_GR);
                                INDEX_GR = 0; PACKET_GR_COMPLETED = false;
                                for (int i = INDEX_GR; i < LENGTH_INT_GR; i++) GLOVE_PACKET_RX[i] = (byte)SP1.ReadByte();
                                INDEX_GR = LENGTH_INT_GR; PACKET_GR_COMPLETED = false;
                            }
                            else { ERROR_RG8++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
                        }
                    }
                }
            }
            catch { ERROR_RG9++; INDEX_GR = 0; LENGTH_INT_GR = 0; }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void PACKET_ASSESSMENT_LG()
        {
            if (((char)GLOVE_PACKET_LX[0] == 'G') && ((char)GLOVE_PACKET_LX[1] == 'L') && ((char)GLOVE_PACKET_LX[2] == 'T'))
            {
                countMicroGL = GLOVE_PACKET_LX[3];
                countMicroLG[0] = countMicroGL;
                if (!Calibration)
                {
                    if (((char)GLOVE_PACKET_LX[PACKET_NUM_G - 3] == 'E') && ((char)GLOVE_PACKET_LX[PACKET_NUM_G - 2] == 'M'))
                    {
                        if (GLOVE_PACKET_LX[3] == GLOVE_PACKET_LX[PACKET_NUM_G - 1])
                        {
                            if (DataLossControlLG)
                            {
                                if ((countMicroLG[0] - countMicroLG[1] > 1) && (countMicroLG[0] > countMicroLG[1]))
                                {
                                    countMicroLossLG = countMicroLossLG + (countMicroLG[0] - countMicroLG[1]) - 1;
                                }
                                else if (countMicroLG[1] > countMicroLG[0])
                                {
                                    countMicroLossLG = countMicroLossLG + (255 - countMicroLG[1] + countMicroLG[0]);
                                }
                            }
                            frequencyLG++; countfreqLG++; countMicroLG[1] = countMicroLG[0];
                            if (!DATA_VM)
                            {
                                accWX_LX = ((float)((Int16)((GLOVE_PACKET_LX[5] << 8) + GLOVE_PACKET_LX[4]))) / 100;
                                accWY_LX = ((float)((Int16)((GLOVE_PACKET_LX[7] << 8) + GLOVE_PACKET_LX[6]))) / 100;
                                accWZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[9] << 8) + GLOVE_PACKET_LX[8]))) / 100;
                                gyrWX_LX = ((float)((Int16)((GLOVE_PACKET_LX[11] << 8) + GLOVE_PACKET_LX[10]))) / 10;
                                gyrWY_LX = ((float)((Int16)((GLOVE_PACKET_LX[13] << 8) + GLOVE_PACKET_LX[12]))) / 10;
                                gyrWZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[15] << 8) + GLOVE_PACKET_LX[14]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magWX_LX = ((float)((Int16)((GLOVE_PACKET_LX[17] << 8) + GLOVE_PACKET_LX[16]))) / 100;
                                    magWY_LX = ((float)((Int16)((GLOVE_PACKET_LX[19] << 8) + GLOVE_PACKET_LX[18]))) / 100;
                                    magWZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[21] << 8) + GLOVE_PACKET_LX[20]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollW_LX = ((float)((Int16)((GLOVE_PACKET_LX[17] << 8) + GLOVE_PACKET_LX[16]))) / 100;
                                    pitchW_LX = ((float)((Int16)((GLOVE_PACKET_LX[19] << 8) + GLOVE_PACKET_LX[18]))) / 100;
                                    yawW_LX = ((float)((Int16)((GLOVE_PACKET_LX[21] << 8) + GLOVE_PACKET_LX[20]))) / 100;
                                }
                                accTX_LX = ((float)((Int16)((GLOVE_PACKET_LX[23] << 8) + GLOVE_PACKET_LX[22]))) / 100;
                                accTY_LX = ((float)((Int16)((GLOVE_PACKET_LX[25] << 8) + GLOVE_PACKET_LX[24]))) / 100;
                                accTZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[27] << 8) + GLOVE_PACKET_LX[26]))) / 100;
                                gyrTX_LX = ((float)((Int16)((GLOVE_PACKET_LX[29] << 8) + GLOVE_PACKET_LX[28]))) / 10;
                                gyrTY_LX = ((float)((Int16)((GLOVE_PACKET_LX[31] << 8) + GLOVE_PACKET_LX[30]))) / 10;
                                gyrTZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[33] << 8) + GLOVE_PACKET_LX[32]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magTX_LX = ((float)((Int16)((GLOVE_PACKET_LX[35] << 8) + GLOVE_PACKET_LX[34]))) / 100;
                                    magTY_LX = ((float)((Int16)((GLOVE_PACKET_LX[37] << 8) + GLOVE_PACKET_LX[36]))) / 100;
                                    magTZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[39] << 8) + GLOVE_PACKET_LX[38]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollT_LX = ((float)((Int16)((GLOVE_PACKET_LX[35] << 8) + GLOVE_PACKET_LX[34]))) / 100;
                                    pitchT_LX = ((float)((Int16)((GLOVE_PACKET_LX[37] << 8) + GLOVE_PACKET_LX[36]))) / 100;
                                    yawT_LX = ((float)((Int16)((GLOVE_PACKET_LX[39] << 8) + GLOVE_PACKET_LX[38]))) / 100;
                                }

                                accIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[41] << 8) + GLOVE_PACKET_LX[40]))) / 100;
                                accIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[43] << 8) + GLOVE_PACKET_LX[42]))) / 100;
                                accIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[45] << 8) + GLOVE_PACKET_LX[44]))) / 100;
                                gyrIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[47] << 8) + GLOVE_PACKET_LX[46]))) / 10;
                                gyrIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[49] << 8) + GLOVE_PACKET_LX[48]))) / 10;
                                gyrIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[51] << 8) + GLOVE_PACKET_LX[50]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[53] << 8) + GLOVE_PACKET_LX[52]))) / 100;
                                    magIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[55] << 8) + GLOVE_PACKET_LX[54]))) / 100;
                                    magIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[57] << 8) + GLOVE_PACKET_LX[56]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollI_LX = ((float)((Int16)((GLOVE_PACKET_LX[53] << 8) + GLOVE_PACKET_LX[52]))) / 100;
                                    pitchI_LX = ((float)((Int16)((GLOVE_PACKET_LX[55] << 8) + GLOVE_PACKET_LX[54]))) / 100;
                                    yawI_LX = ((float)((Int16)((GLOVE_PACKET_LX[57] << 8) + GLOVE_PACKET_LX[56]))) / 100;
                                }

                                accMX_LX = ((float)((Int16)((GLOVE_PACKET_LX[59] << 8) + GLOVE_PACKET_LX[58]))) / 100;
                                accMY_LX = ((float)((Int16)((GLOVE_PACKET_LX[61] << 8) + GLOVE_PACKET_LX[60]))) / 100;
                                accMZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[63] << 8) + GLOVE_PACKET_LX[62]))) / 100;
                                gyrMX_LX = ((float)((Int16)((GLOVE_PACKET_LX[65] << 8) + GLOVE_PACKET_LX[64]))) / 10;
                                gyrMY_LX = ((float)((Int16)((GLOVE_PACKET_LX[67] << 8) + GLOVE_PACKET_LX[66]))) / 10;
                                gyrMZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[69] << 8) + GLOVE_PACKET_LX[68]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magMX_LX = ((float)((Int16)((GLOVE_PACKET_LX[71] << 8) + GLOVE_PACKET_LX[70]))) / 100;
                                    magMY_LX = ((float)((Int16)((GLOVE_PACKET_LX[73] << 8) + GLOVE_PACKET_LX[72]))) / 100;
                                    magMZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[75] << 8) + GLOVE_PACKET_LX[74]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollM_LX = ((float)((Int16)((GLOVE_PACKET_LX[71] << 8) + GLOVE_PACKET_LX[70]))) / 100;
                                    pitchM_LX = ((float)((Int16)((GLOVE_PACKET_LX[73] << 8) + GLOVE_PACKET_LX[72]))) / 100;
                                    yawM_LX = ((float)((Int16)((GLOVE_PACKET_LX[75] << 8) + GLOVE_PACKET_LX[74]))) / 100;
                                }

                                //this.Invoke(new UpdateGUI(this.UpdatePanel));
                                UpdatePanel(workMode);

                                if (path_existHLX)
                                {
                                    accWX_LX_txt = String.Format("{0:000.0000}", accWX_LX).ToString();
                                    accWY_LX_txt = String.Format("{0:000.0000}", accWY_LX).ToString();
                                    accWZ_LX_txt = String.Format("{0:000.0000}", accWZ_LX).ToString();

                                    accTX_LX_txt = String.Format("{0:000.0000}", accTX_LX).ToString();
                                    accTY_LX_txt = String.Format("{0:000.0000}", accTY_LX).ToString();
                                    accTZ_LX_txt = String.Format("{0:000.0000}", accTZ_LX).ToString();

                                    accIX_LX_txt = String.Format("{0:000.0000}", accIX_LX).ToString();
                                    accIY_LX_txt = String.Format("{0:000.0000}", accIY_LX).ToString();
                                    accIZ_LX_txt = String.Format("{0:000.0000}", accIZ_LX).ToString();

                                    accMX_LX_txt = String.Format("{0:000.0000}", accMX_LX).ToString();
                                    accMY_LX_txt = String.Format("{0:000.0000}", accMY_LX).ToString();
                                    accMZ_LX_txt = String.Format("{0:000.0000}", accMZ_LX).ToString();

                                    gyrWX_LX_txt = String.Format("{0:000.0000}", gyrWX_LX).ToString();
                                    gyrWY_LX_txt = String.Format("{0:000.0000}", gyrWY_LX).ToString();
                                    gyrWZ_LX_txt = String.Format("{0:000.0000}", gyrWZ_LX).ToString();

                                    gyrTX_LX_txt = String.Format("{0:000.0000}", gyrTX_LX).ToString();
                                    gyrTY_LX_txt = String.Format("{0:000.0000}", gyrTY_LX).ToString();
                                    gyrTZ_LX_txt = String.Format("{0:000.0000}", gyrTZ_LX).ToString();

                                    gyrIX_LX_txt = String.Format("{0:000.0000}", gyrIX_LX).ToString();
                                    gyrIY_LX_txt = String.Format("{0:000.0000}", gyrIY_LX).ToString();
                                    gyrIZ_LX_txt = String.Format("{0:000.0000}", gyrIZ_LX).ToString();

                                    gyrMX_LX_txt = String.Format("{0:000.0000}", gyrMX_LX).ToString();
                                    gyrMY_LX_txt = String.Format("{0:000.0000}", gyrMY_LX).ToString();
                                    gyrMZ_LX_txt = String.Format("{0:000.0000}", gyrMZ_LX).ToString();

                                    if ((DATA_LOG) || (DATA_GRAPH))
                                    {
                                        magWX_LX_txt = String.Format("{0:000.0000}", magWX_LX).ToString();
                                        magWY_LX_txt = String.Format("{0:000.0000}", magWY_LX).ToString();
                                        magWZ_LX_txt = String.Format("{0:000.0000}", magWZ_LX).ToString();

                                        magTX_LX_txt = String.Format("{0:000.0000}", magTX_LX).ToString();
                                        magTY_LX_txt = String.Format("{0:000.0000}", magTY_LX).ToString();
                                        magTZ_LX_txt = String.Format("{0:000.0000}", magTZ_LX).ToString();

                                        magIX_LX_txt = String.Format("{0:000.0000}", magIX_LX).ToString();
                                        magIY_LX_txt = String.Format("{0:000.0000}", magIY_LX).ToString();
                                        magIZ_LX_txt = String.Format("{0:000.0000}", magIZ_LX).ToString();

                                        magMX_LX_txt = String.Format("{0:000.0000}", magMX_LX).ToString();
                                        magMY_LX_txt = String.Format("{0:000.0000}", magMY_LX).ToString();
                                        magMZ_LX_txt = String.Format("{0:000.0000}", magMZ_LX).ToString();
                                    }
                                    else if (DATA_AHRS)
                                    {
                                        rollW_LX_txt = String.Format("{0:000.0000}", rollW_LX).ToString();
                                        pitchW_LX_txt = String.Format("{0:000.0000}", pitchW_LX).ToString();
                                        yawW_LX_txt = String.Format("{0:000.0000}", yawW_LX).ToString();

                                        rollT_LX_txt = String.Format("{0:000.0000}", rollT_LX).ToString();
                                        pitchT_LX_txt = String.Format("{0:000.0000}", pitchT_LX).ToString();
                                        yawT_LX_txt = String.Format("{0:000.0000}", yawT_LX).ToString();

                                        rollI_LX_txt = String.Format("{0:000.0000}", rollI_LX).ToString();
                                        pitchI_LX_txt = String.Format("{0:000.0000}", pitchI_LX).ToString();
                                        yawI_LX_txt = String.Format("{0:000.0000}", yawI_LX).ToString();

                                        rollM_LX_txt = String.Format("{0:000.0000}", rollM_LX).ToString();
                                        pitchM_LX_txt = String.Format("{0:000.0000}", pitchM_LX).ToString();
                                        yawM_LX_txt = String.Format("{0:000.0000}", yawM_LX).ToString();
                                    }

                                    StreamWriter FileDATA_LG = File.AppendText(pathHLX);
                                    FileDATA_LG.Flush();
                                    if ((DATA_LOG) || (DATA_GRAPH))
                                    {
                                        report = countfreqLG.ToString() +
                                       "\t" + accWX_LX_txt + "\t" + accWY_LX_txt + "\t" + accWZ_LX_txt + "\t" + gyrWX_LX_txt + "\t" + gyrWY_LX_txt + "\t" + gyrWZ_LX_txt + "\t" + magWX_LX_txt + "\t" + magWY_LX_txt + "\t" + magWZ_LX_txt +
                                       "\t" + accMX_LX_txt + "\t" + accMY_LX_txt + "\t" + accMZ_LX_txt + "\t" + gyrMX_LX_txt + "\t" + gyrMY_LX_txt + "\t" + gyrMZ_LX_txt + "\t" + magMX_LX_txt + "\t" + magMY_LX_txt + "\t" + magMZ_LX_txt +
                                       "\t" + accIX_LX_txt + "\t" + accIY_LX_txt + "\t" + accIZ_LX_txt + "\t" + gyrIX_LX_txt + "\t" + gyrIY_LX_txt + "\t" + gyrIZ_LX_txt + "\t" + magIX_LX_txt + "\t" + magIY_LX_txt + "\t" + magIZ_LX_txt +
                                       "\t" + accTX_LX_txt + "\t" + accTY_LX_txt + "\t" + accTZ_LX_txt + "\t" + gyrTX_LX_txt + "\t" + gyrTY_LX_txt + "\t" + gyrTZ_LX_txt + "\t" + magTX_LX_txt + "\t" + magTY_LX_txt + "\t" + magTZ_LX_txt +
                                       "\t" + countMicroGL + "\r\n";
                                    }
                                    else if (DATA_AHRS)
                                    {
                                        report = countfreqLG.ToString() +
                                       "\t" + accWX_LX_txt + "\t" + accWY_LX_txt + "\t" + accWZ_LX_txt + "\t" + gyrWX_LX_txt + "\t" + gyrWY_LX_txt + "\t" + gyrWZ_LX_txt + "\t" + rollW_LX_txt + "\t" + pitchW_LX_txt + "\t" + yawW_LX_txt +
                                       "\t" + accMX_LX_txt + "\t" + accMY_LX_txt + "\t" + accMZ_LX_txt + "\t" + gyrMX_LX_txt + "\t" + gyrMY_LX_txt + "\t" + gyrMZ_LX_txt + "\t" + rollM_LX_txt + "\t" + pitchM_LX_txt + "\t" + yawM_LX_txt +
                                       "\t" + accIX_LX_txt + "\t" + accIY_LX_txt + "\t" + accIZ_LX_txt + "\t" + gyrIX_LX_txt + "\t" + gyrIY_LX_txt + "\t" + gyrIZ_LX_txt + "\t" + rollI_LX_txt + "\t" + pitchI_LX_txt + "\t" + yawI_LX_txt +
                                       "\t" + accTX_LX_txt + "\t" + accTY_LX_txt + "\t" + accTZ_LX_txt + "\t" + gyrTX_LX_txt + "\t" + gyrTY_LX_txt + "\t" + gyrTZ_LX_txt + "\t" + rollT_LX_txt + "\t" + pitchT_LX_txt + "\t" + yawT_LX_txt +
                                       "\t" + countMicroGL + "\r\n";
                                    }
                                    FileDATA_LG.WriteLine(report); FileDATA_LG.Flush(); FileDATA_LG.Close();
                                }
                                PACKET_GL_COMPLETED = true;
                            }
                            else
                            {
                                accIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[5] << 8) + GLOVE_PACKET_LX[4]))) / 100;
                                accIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[7] << 8) + GLOVE_PACKET_LX[6]))) / 100;
                                accIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[9] << 8) + GLOVE_PACKET_LX[8]))) / 100;
                                gyrIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[11] << 8) + GLOVE_PACKET_LX[10]))) / 10;
                                gyrIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[13] << 8) + GLOVE_PACKET_LX[12]))) / 10;
                                gyrIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[15] << 8) + GLOVE_PACKET_LX[14]))) / 10;
                                magIX_LX = ((float)((Int16)((GLOVE_PACKET_LX[17] << 8) + GLOVE_PACKET_LX[16]))) / 100;
                                magIY_LX = ((float)((Int16)((GLOVE_PACKET_LX[19] << 8) + GLOVE_PACKET_LX[18]))) / 100;
                                magIZ_LX = ((float)((Int16)((GLOVE_PACKET_LX[21] << 8) + GLOVE_PACKET_LX[20]))) / 100;
                                PACKET_GL_COMPLETED = true;
                                UpdatePanel(workMode);

                            }
                        }
                    }
                }
            }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void PACKET_ASSESSMENT_RG()
        {
            if (((char)GLOVE_PACKET_RX[0] == 'G') && ((char)GLOVE_PACKET_RX[1] == 'R') && ((char)GLOVE_PACKET_RX[2] == 'H'))
            {
                countMicroGR = GLOVE_PACKET_RX[3];
                countMicroRG[0] = countMicroGR;
                if (!Calibration)
                {
                    if (((char)GLOVE_PACKET_RX[PACKET_NUM_G - 3] == 'E') && ((char)GLOVE_PACKET_RX[PACKET_NUM_G - 2] == 'N'))
                    {
                        if (GLOVE_PACKET_RX[3] == GLOVE_PACKET_RX[PACKET_NUM_G - 1])
                        {
                            if (DataLossControlRG)
                            {
                                if ((countMicroRG[0] - countMicroRG[1] > 1) && (countMicroRG[0] > countMicroRG[1]))
                                {
                                    countMicroLossRG = countMicroLossRG + (countMicroRG[0] - countMicroRG[1]) - 1;
                                }
                                else if (countMicroRG[1] > countMicroRG[0])
                                {
                                    countMicroLossRG = countMicroLossRG + (255 - countMicroRG[1] + countMicroRG[0]);
                                }
                            }
                            frequencyRG++; countfreqRG++; countMicroRG[1] = countMicroRG[0];
                            if (!DATA_VM)
                            {
                                accWX_RX = ((float)((Int16)((GLOVE_PACKET_RX[5] << 8) + GLOVE_PACKET_RX[4]))) / 100;
                                accWY_RX = ((float)((Int16)((GLOVE_PACKET_RX[7] << 8) + GLOVE_PACKET_RX[6]))) / 100;
                                accWZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[9] << 8) + GLOVE_PACKET_RX[8]))) / 100;
                                gyrWX_RX = ((float)((Int16)((GLOVE_PACKET_RX[11] << 8) + GLOVE_PACKET_RX[10]))) / 10;
                                gyrWY_RX = ((float)((Int16)((GLOVE_PACKET_RX[13] << 8) + GLOVE_PACKET_RX[12]))) / 10;
                                gyrWZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[15] << 8) + GLOVE_PACKET_RX[14]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magWX_RX = ((float)((Int16)((GLOVE_PACKET_RX[17] << 8) + GLOVE_PACKET_RX[16]))) / 100;
                                    magWY_RX = ((float)((Int16)((GLOVE_PACKET_RX[19] << 8) + GLOVE_PACKET_RX[18]))) / 100;
                                    magWZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[21] << 8) + GLOVE_PACKET_RX[20]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollW_RX = ((float)((Int16)((GLOVE_PACKET_RX[17] << 8) + GLOVE_PACKET_RX[16]))) / 100;
                                    pitchW_RX = ((float)((Int16)((GLOVE_PACKET_RX[19] << 8) + GLOVE_PACKET_RX[18]))) / 100;
                                    yawW_RX = ((float)((Int16)((GLOVE_PACKET_RX[21] << 8) + GLOVE_PACKET_RX[20]))) / 100;
                                }
                                accTX_RX = ((float)((Int16)((GLOVE_PACKET_RX[23] << 8) + GLOVE_PACKET_RX[22]))) / 100;
                                accTY_RX = ((float)((Int16)((GLOVE_PACKET_RX[25] << 8) + GLOVE_PACKET_RX[24]))) / 100;
                                accTZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[27] << 8) + GLOVE_PACKET_RX[26]))) / 100;
                                gyrTX_RX = ((float)((Int16)((GLOVE_PACKET_RX[29] << 8) + GLOVE_PACKET_RX[28]))) / 10;
                                gyrTY_RX = ((float)((Int16)((GLOVE_PACKET_RX[31] << 8) + GLOVE_PACKET_RX[30]))) / 10;
                                gyrTZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[33] << 8) + GLOVE_PACKET_RX[32]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magTX_RX = ((float)((Int16)((GLOVE_PACKET_RX[35] << 8) + GLOVE_PACKET_RX[34]))) / 100;
                                    magTY_RX = ((float)((Int16)((GLOVE_PACKET_RX[37] << 8) + GLOVE_PACKET_RX[36]))) / 100;
                                    magTZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[39] << 8) + GLOVE_PACKET_RX[38]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollT_RX = ((float)((Int16)((GLOVE_PACKET_RX[35] << 8) + GLOVE_PACKET_RX[34]))) / 100;
                                    pitchT_RX = ((float)((Int16)((GLOVE_PACKET_RX[37] << 8) + GLOVE_PACKET_RX[36]))) / 100;
                                    yawT_RX = ((float)((Int16)((GLOVE_PACKET_RX[39] << 8) + GLOVE_PACKET_RX[38]))) / 100;
                                }

                                accIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[41] << 8) + GLOVE_PACKET_RX[40]))) / 100;
                                accIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[43] << 8) + GLOVE_PACKET_RX[42]))) / 100;
                                accIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[45] << 8) + GLOVE_PACKET_RX[44]))) / 100;
                                gyrIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[47] << 8) + GLOVE_PACKET_RX[46]))) / 10;
                                gyrIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[49] << 8) + GLOVE_PACKET_RX[48]))) / 10;
                                gyrIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[51] << 8) + GLOVE_PACKET_RX[50]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[53] << 8) + GLOVE_PACKET_RX[52]))) / 100;
                                    magIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[55] << 8) + GLOVE_PACKET_RX[54]))) / 100;
                                    magIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[57] << 8) + GLOVE_PACKET_RX[56]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollI_RX = ((float)((Int16)((GLOVE_PACKET_RX[53] << 8) + GLOVE_PACKET_RX[52]))) / 100;
                                    pitchI_RX = ((float)((Int16)((GLOVE_PACKET_RX[55] << 8) + GLOVE_PACKET_RX[54]))) / 100;
                                    yawI_RX = ((float)((Int16)((GLOVE_PACKET_RX[57] << 8) + GLOVE_PACKET_RX[56]))) / 100;
                                }

                                accMX_RX = ((float)((Int16)((GLOVE_PACKET_RX[59] << 8) + GLOVE_PACKET_RX[58]))) / 100;
                                accMY_RX = ((float)((Int16)((GLOVE_PACKET_RX[61] << 8) + GLOVE_PACKET_RX[60]))) / 100;
                                accMZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[63] << 8) + GLOVE_PACKET_RX[62]))) / 100;
                                gyrMX_RX = ((float)((Int16)((GLOVE_PACKET_RX[65] << 8) + GLOVE_PACKET_RX[64]))) / 10;
                                gyrMY_RX = ((float)((Int16)((GLOVE_PACKET_RX[67] << 8) + GLOVE_PACKET_RX[66]))) / 10;
                                gyrMZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[69] << 8) + GLOVE_PACKET_RX[68]))) / 10;
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magMX_RX = ((float)((Int16)((GLOVE_PACKET_RX[71] << 8) + GLOVE_PACKET_RX[70]))) / 100;
                                    magMY_RX = ((float)((Int16)((GLOVE_PACKET_RX[73] << 8) + GLOVE_PACKET_RX[72]))) / 100;
                                    magMZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[75] << 8) + GLOVE_PACKET_RX[74]))) / 100;
                                }
                                else if (DATA_AHRS)
                                {
                                    rollM_RX = ((float)((Int16)((GLOVE_PACKET_RX[71] << 8) + GLOVE_PACKET_RX[70]))) / 100;
                                    pitchM_RX = ((float)((Int16)((GLOVE_PACKET_RX[73] << 8) + GLOVE_PACKET_RX[72]))) / 100;
                                    yawM_RX = ((float)((Int16)((GLOVE_PACKET_RX[75] << 8) + GLOVE_PACKET_RX[74]))) / 100;
                                }

                                UpdatePanel(workMode);

                                if (path_existHRX)
                                {
                                    accWX_RX_txt = String.Format("{0:000.0000}", accWX_RX).ToString();
                                    accWY_RX_txt = String.Format("{0:000.0000}", accWY_RX).ToString();
                                    accWZ_RX_txt = String.Format("{0:000.0000}", accWZ_RX).ToString();

                                    accTX_RX_txt = String.Format("{0:000.0000}", accTX_RX).ToString();
                                    accTY_RX_txt = String.Format("{0:000.0000}", accTY_RX).ToString();
                                    accTZ_RX_txt = String.Format("{0:000.0000}", accTZ_RX).ToString();

                                    accIX_RX_txt = String.Format("{0:000.0000}", accIX_RX).ToString();
                                    accIY_RX_txt = String.Format("{0:000.0000}", accIY_RX).ToString();
                                    accIZ_RX_txt = String.Format("{0:000.0000}", accIZ_RX).ToString();

                                    accMX_RX_txt = String.Format("{0:000.0000}", accMX_RX).ToString();
                                    accMY_RX_txt = String.Format("{0:000.0000}", accMY_RX).ToString();
                                    accMZ_RX_txt = String.Format("{0:000.0000}", accMZ_RX).ToString();

                                    gyrWX_RX_txt = String.Format("{0:000.0000}", gyrWX_RX).ToString();
                                    gyrWY_RX_txt = String.Format("{0:000.0000}", gyrWY_RX).ToString();
                                    gyrWZ_RX_txt = String.Format("{0:000.0000}", gyrWZ_RX).ToString();

                                    gyrTX_RX_txt = String.Format("{0:000.0000}", gyrTX_RX).ToString();
                                    gyrTY_RX_txt = String.Format("{0:000.0000}", gyrTY_RX).ToString();
                                    gyrTZ_RX_txt = String.Format("{0:000.0000}", gyrTZ_RX).ToString();

                                    gyrIX_RX_txt = String.Format("{0:000.0000}", gyrIX_RX).ToString();
                                    gyrIY_RX_txt = String.Format("{0:000.0000}", gyrIY_RX).ToString();
                                    gyrIZ_RX_txt = String.Format("{0:000.0000}", gyrIZ_RX).ToString();

                                    gyrMX_RX_txt = String.Format("{0:000.0000}", gyrMX_RX).ToString();
                                    gyrMY_RX_txt = String.Format("{0:000.0000}", gyrMY_RX).ToString();
                                    gyrMZ_RX_txt = String.Format("{0:000.0000}", gyrMZ_RX).ToString();

                                    if ((DATA_LOG) || (DATA_GRAPH))
                                    {
                                        magWX_RX_txt = String.Format("{0:000.0000}", magWX_RX).ToString();
                                        magWY_RX_txt = String.Format("{0:000.0000}", magWY_RX).ToString();
                                        magWZ_RX_txt = String.Format("{0:000.0000}", magWZ_RX).ToString();

                                        magTX_RX_txt = String.Format("{0:000.0000}", magTX_RX).ToString();
                                        magTY_RX_txt = String.Format("{0:000.0000}", magTY_RX).ToString();
                                        magTZ_RX_txt = String.Format("{0:000.0000}", magTZ_RX).ToString();

                                        magIX_RX_txt = String.Format("{0:000.0000}", magIX_RX).ToString();
                                        magIY_RX_txt = String.Format("{0:000.0000}", magIY_RX).ToString();
                                        magIZ_RX_txt = String.Format("{0:000.0000}", magIZ_RX).ToString();

                                        magMX_RX_txt = String.Format("{0:000.0000}", magMX_RX).ToString();
                                        magMY_RX_txt = String.Format("{0:000.0000}", magMY_RX).ToString();
                                        magMZ_RX_txt = String.Format("{0:000.0000}", magMZ_RX).ToString();
                                    }
                                    else if (DATA_AHRS)
                                    {
                                        rollW_RX_txt = String.Format("{0:000.0000}", rollW_RX).ToString();
                                        pitchW_RX_txt = String.Format("{0:000.0000}", pitchW_RX).ToString();
                                        yawW_RX_txt = String.Format("{0:000.0000}", yawW_RX).ToString();

                                        rollT_RX_txt = String.Format("{0:000.0000}", rollT_RX).ToString();
                                        pitchT_RX_txt = String.Format("{0:000.0000}", pitchT_RX).ToString();
                                        yawT_RX_txt = String.Format("{0:000.0000}", yawT_RX).ToString();

                                        rollI_RX_txt = String.Format("{0:000.0000}", rollI_RX).ToString();
                                        pitchI_RX_txt = String.Format("{0:000.0000}", pitchI_RX).ToString();
                                        yawI_RX_txt = String.Format("{0:000.0000}", yawI_RX).ToString();

                                        rollM_RX_txt = String.Format("{0:000.0000}", rollM_RX).ToString();
                                        pitchM_RX_txt = String.Format("{0:000.0000}", pitchM_RX).ToString();
                                        yawM_RX_txt = String.Format("{0:000.0000}", yawM_RX).ToString();
                                    }

                                    StreamWriter FileDATA_RG = File.AppendText(pathHRX);
                                    FileDATA_RG.Flush();
                                    if ((DATA_LOG) || (DATA_GRAPH))
                                    {
                                        report = countfreqRG.ToString() +
                                       "\t" + accWX_RX_txt + "\t" + accWY_RX_txt + "\t" + accWZ_RX_txt + "\t" + gyrWX_RX_txt + "\t" + gyrWY_RX_txt + "\t" + gyrWZ_RX_txt + "\t" + magWX_RX_txt + "\t" + magWY_RX_txt + "\t" + magWZ_RX_txt +
                                       "\t" + accTX_RX_txt + "\t" + accTY_RX_txt + "\t" + accTZ_RX_txt + "\t" + gyrTX_RX_txt + "\t" + gyrTY_RX_txt + "\t" + gyrTZ_RX_txt + "\t" + magTX_RX_txt + "\t" + magTY_RX_txt + "\t" + magTZ_RX_txt +
                                       "\t" + accIX_RX_txt + "\t" + accIY_RX_txt + "\t" + accIZ_RX_txt + "\t" + gyrIX_RX_txt + "\t" + gyrIY_RX_txt + "\t" + gyrIZ_RX_txt + "\t" + magIX_RX_txt + "\t" + magIY_RX_txt + "\t" + magIZ_RX_txt +
                                       "\t" + accMX_RX_txt + "\t" + accMY_RX_txt + "\t" + accMZ_RX_txt + "\t" + gyrMX_RX_txt + "\t" + gyrMY_RX_txt + "\t" + gyrMZ_RX_txt + "\t" + magMX_RX_txt + "\t" + magMY_RX_txt + "\t" + magMZ_RX_txt +
                                       "\t" + countMicroGR + "\r\n";
                                    }
                                    else if (DATA_AHRS)
                                    {
                                        report = countfreqRG.ToString() +
                                       "\t" + accWX_RX_txt + "\t" + accWY_RX_txt + "\t" + accWZ_RX_txt + "\t" + gyrWX_RX_txt + "\t" + gyrWY_RX_txt + "\t" + gyrWZ_RX_txt + "\t" + rollW_LX_txt + "\t" + pitchW_RX_txt + "\t" + yawW_RX_txt +
                                       "\t" + accTX_RX_txt + "\t" + accTY_RX_txt + "\t" + accTZ_RX_txt + "\t" + gyrTX_RX_txt + "\t" + gyrTY_RX_txt + "\t" + gyrTZ_RX_txt + "\t" + rollT_RX_txt + "\t" + pitchT_RX_txt + "\t" + yawT_RX_txt +
                                       "\t" + accIX_RX_txt + "\t" + accIY_RX_txt + "\t" + accIZ_RX_txt + "\t" + gyrIX_RX_txt + "\t" + gyrIY_RX_txt + "\t" + gyrIZ_RX_txt + "\t" + rollI_RX_txt + "\t" + pitchI_RX_txt + "\t" + yawI_RX_txt +
                                       "\t" + accMX_RX_txt + "\t" + accMY_RX_txt + "\t" + accMZ_RX_txt + "\t" + gyrMX_RX_txt + "\t" + gyrMY_RX_txt + "\t" + gyrMZ_RX_txt + "\t" + rollM_RX_txt + "\t" + pitchM_RX_txt + "\t" + yawM_RX_txt +
                                       "\t" + countMicroGR + "\r\n";
                                    }
                                    FileDATA_RG.WriteLine(report); FileDATA_RG.Flush(); FileDATA_RG.Close();
                                }
                                PACKET_GR_COMPLETED = true;
                            }
                            else
                            {
                                accIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[5] << 8) + GLOVE_PACKET_RX[4]))) / 100;
                                accIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[7] << 8) + GLOVE_PACKET_RX[6]))) / 100;
                                accIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[9] << 8) + GLOVE_PACKET_RX[8]))) / 100;
                                gyrIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[11] << 8) + GLOVE_PACKET_RX[10]))) / 10;
                                gyrIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[13] << 8) + GLOVE_PACKET_RX[12]))) / 10;
                                gyrIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[15] << 8) + GLOVE_PACKET_RX[14]))) / 10;
                                magIX_RX = ((float)((Int16)((GLOVE_PACKET_RX[17] << 8) + GLOVE_PACKET_RX[16]))) / 100;
                                magIY_RX = ((float)((Int16)((GLOVE_PACKET_RX[19] << 8) + GLOVE_PACKET_RX[18]))) / 100;
                                magIZ_RX = ((float)((Int16)((GLOVE_PACKET_RX[21] << 8) + GLOVE_PACKET_RX[20]))) / 100;
                                PACKET_GR_COMPLETED = true;
                                UpdatePanel(workMode);

                            }
                        }
                    }
                }
            }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void FOOT_PACKET_READ_LX()
        {
            char[] LENGTH_CHAR_FL = new char[3];
            LENGTH_CHAR_FL[0] = (char)SP1.ReadByte();
            for (int i = 1; i < 3; i++) LENGTH_CHAR_FL[i] = (char)SP1.ReadByte();
            string LENGTH_FL = string.Concat(LENGTH_CHAR_FL);
            int LENGTH_INT_FL = 0;
            try
            {
                LENGTH_INT_FL = int.Parse(LENGTH_FL);
                if (INDEX_FL == 0)
                {
                    FOOT_PACKET_LX[0] = (byte)SP1.ReadByte();
                    if ((char)FOOT_PACKET_LX[0] == 'L')
                    {
                        if (LENGTH_INT_FL >= PACKET_NUM_F)
                        {

                            for (int i = 1; i < 3; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            if (((char)FOOT_PACKET_LX[1] == 'T') && ((char)FOOT_PACKET_LX[2] == 'F'))
                            {
                                INDEX_FL = INDEX_FL + 3;
                                while (LENGTH_INT_FL >= PACKET_NUM_F)
                                {
                                    for (int i = INDEX_FL; i < PACKET_NUM_F; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                                    PACKET_ASSESSMENT_LF();
                                    if (PACKET_FL_COMPLETED)
                                    {
                                        LENGTH_INT_FL = LENGTH_INT_FL - PACKET_NUM_F;
                                        INDEX_FL = 0; PACKET_FL_COMPLETED = false;
                                        if ((LENGTH_INT_FL > 0) && (LENGTH_INT_FL < PACKET_NUM_F))
                                        {
                                            FOOT_PACKET_LX[0] = (byte)SP1.ReadByte();
                                            if ((char)FOOT_PACKET_LX[0] == 'L')
                                            {
                                                INDEX_FL++;
                                                for (int i = INDEX_FL; i < LENGTH_INT_FL; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                                                INDEX_FL = LENGTH_INT_FL; PACKET_FL_COMPLETED = false;
                                            }
                                            else { ERROR_LF1++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                                        }
                                    }
                                    else { ERROR_LF2++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                                }
                            }
                            else { ERROR_LF3++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                        }
                        else if (LENGTH_INT_FL < PACKET_NUM_F)
                        {
                            INDEX_FL++;
                            for (int i = INDEX_FL; i < LENGTH_INT_FL; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            INDEX_FL = LENGTH_INT_FL; PACKET_FL_COMPLETED = false;
                        }
                    }
                    else { ERROR_LF4++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                }
                else if (INDEX_FL > 0)
                {
                    if (LENGTH_INT_FL >= PACKET_NUM_F)
                    {
                        while (LENGTH_INT_FL >= PACKET_NUM_F)
                        {
                            for (int i = INDEX_FL; i < PACKET_NUM_F; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LF();
                            if (PACKET_FL_COMPLETED)
                            {
                                LENGTH_INT_FL = LENGTH_INT_FL - (PACKET_NUM_F - INDEX_FL);
                                INDEX_FL = 0; PACKET_FL_COMPLETED = false;
                                if ((LENGTH_INT_FL > 0) && (LENGTH_INT_FL < PACKET_NUM_F))
                                {
                                    FOOT_PACKET_LX[0] = (byte)SP1.ReadByte();
                                    if ((char)FOOT_PACKET_LX[0] == 'L')
                                    {
                                        INDEX_FL++;
                                        for (int i = INDEX_FL; i < LENGTH_INT_FL; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                                        INDEX_FL = LENGTH_INT_FL; PACKET_FL_COMPLETED = false;
                                    }
                                    else { ERROR_LF5++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                                }
                            }
                            else { ERROR_LF6++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                        }
                    }
                    else if (LENGTH_INT_FL < PACKET_NUM_F)
                    {
                        if (LENGTH_INT_FL + INDEX_FL == PACKET_NUM_F)
                        {
                            for (int i = INDEX_FL; i < PACKET_NUM_F; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LF();
                            if (PACKET_FL_COMPLETED) { INDEX_FL = 0; PACKET_FL_COMPLETED = false; }
                            else { ERROR_LF7++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                        }
                        else if (LENGTH_INT_FL + INDEX_FL < PACKET_NUM_F)
                        {
                            for (int i = INDEX_FL; i < LENGTH_INT_FL + INDEX_FL; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            INDEX_FL = LENGTH_INT_FL + INDEX_FL; PACKET_FL_COMPLETED = false;
                        }
                        else if (LENGTH_INT_FL + INDEX_FL > PACKET_NUM_F)
                        {
                            for (int i = INDEX_FL; i < PACKET_NUM_F; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_LF();
                            if (PACKET_FL_COMPLETED)
                            {
                                LENGTH_INT_FL = LENGTH_INT_FL - (PACKET_NUM_F - INDEX_FL);
                                INDEX_FL = 0; PACKET_FL_COMPLETED = false;
                                for (int i = INDEX_FL; i < LENGTH_INT_FL; i++) FOOT_PACKET_LX[i] = (byte)SP1.ReadByte();
                                INDEX_FL = LENGTH_INT_FL; PACKET_FL_COMPLETED = false;
                            }
                            else { ERROR_LF8++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
                        }
                    }
                }
            }
            catch { ERROR_LF9++; INDEX_FL = 0; LENGTH_INT_FL = 0; }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void FOOT_PACKET_READ_RX()
        {
            char[] LENGTH_CHAR_FR = new char[3];
            LENGTH_CHAR_FR[0] = (char)SP1.ReadByte();
            for (int i = 1; i < 3; i++) LENGTH_CHAR_FR[i] = (char)SP1.ReadByte();
            string LENGTH_FR = string.Concat(LENGTH_CHAR_FR);
            int LENGTH_INT_FR = 0;
            try
            {
                LENGTH_INT_FR = int.Parse(LENGTH_FR);
                if (INDEX_FR == 0)
                {
                    FOOT_PACKET_RX[0] = (byte)SP1.ReadByte();
                    if ((char)FOOT_PACKET_RX[0] == 'R')
                    {
                        if (LENGTH_INT_FR >= PACKET_NUM_F)
                        {

                            for (int i = 1; i < 3; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            if (((char)FOOT_PACKET_RX[1] == 'H') && ((char)FOOT_PACKET_RX[2] == 'F'))
                            {
                                INDEX_FR = INDEX_FR + 3;
                                while (LENGTH_INT_FR >= PACKET_NUM_F)
                                {
                                    for (int i = INDEX_FR; i < PACKET_NUM_F; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                                    PACKET_ASSESSMENT_RF();
                                    if (PACKET_FR_COMPLETED)
                                    {
                                        LENGTH_INT_FR = LENGTH_INT_FR - PACKET_NUM_F;
                                        INDEX_FR = 0; PACKET_FR_COMPLETED = false;
                                        if ((LENGTH_INT_FR > 0) && (LENGTH_INT_FR < PACKET_NUM_F))
                                        {
                                            FOOT_PACKET_RX[0] = (byte)SP1.ReadByte();
                                            if ((char)FOOT_PACKET_RX[0] == 'R')
                                            {
                                                INDEX_FR++;
                                                for (int i = INDEX_FR; i < LENGTH_INT_FR; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                                                INDEX_FR = LENGTH_INT_FR; PACKET_FR_COMPLETED = false;
                                            }
                                            else { ERROR_RF1++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                                        }
                                    }
                                    else { ERROR_RF2++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                                }
                            }
                            else { ERROR_RF3++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                        }
                        else if (LENGTH_INT_FR < PACKET_NUM_F)
                        {
                            INDEX_FR++;
                            for (int i = INDEX_FR; i < LENGTH_INT_FR; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            INDEX_FR = LENGTH_INT_FR; PACKET_FR_COMPLETED = false;
                        }
                    }
                    else { ERROR_RF4++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                }
                else if (INDEX_FR > 0)
                {
                    if (LENGTH_INT_FR >= PACKET_NUM_F)
                    {
                        while (LENGTH_INT_FR >= PACKET_NUM_F)
                        {
                            for (int i = INDEX_FR; i < PACKET_NUM_F; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RF();
                            if (PACKET_FR_COMPLETED)
                            {
                                LENGTH_INT_FR = LENGTH_INT_FR - (PACKET_NUM_F - INDEX_FR);
                                INDEX_FR = 0; PACKET_FR_COMPLETED = false;
                                if ((LENGTH_INT_FR > 0) && (LENGTH_INT_FR < PACKET_NUM_F))
                                {
                                    FOOT_PACKET_RX[0] = (byte)SP1.ReadByte();
                                    if ((char)FOOT_PACKET_RX[0] == 'R')
                                    {
                                        INDEX_FR++;
                                        for (int i = INDEX_FR; i < LENGTH_INT_FR; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                                        INDEX_FR = LENGTH_INT_FR; PACKET_FR_COMPLETED = false;
                                    }
                                    else { ERROR_RF5++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                                }
                            }
                            else { ERROR_RF6++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                        }
                    }
                    else if (LENGTH_INT_FR < PACKET_NUM_F)
                    {
                        if (LENGTH_INT_FR + INDEX_FR == PACKET_NUM_F)
                        {
                            for (int i = INDEX_FR; i < PACKET_NUM_F; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RF();
                            if (PACKET_FR_COMPLETED) { INDEX_FR = 0; PACKET_FR_COMPLETED = false; }
                            else { ERROR_RF7++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                        }
                        else if (LENGTH_INT_FR + INDEX_FR < PACKET_NUM_F)
                        {
                            for (int i = INDEX_FR; i < LENGTH_INT_FR + INDEX_FR; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            INDEX_FR = LENGTH_INT_FR + INDEX_FR; PACKET_FR_COMPLETED = false;
                        }
                        else if (LENGTH_INT_FR + INDEX_FR > PACKET_NUM_F)
                        {
                            for (int i = INDEX_FR; i < PACKET_NUM_F; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                            PACKET_ASSESSMENT_RF();
                            if (PACKET_FR_COMPLETED)
                            {
                                LENGTH_INT_FR = LENGTH_INT_FR - (PACKET_NUM_F - INDEX_FR);
                                INDEX_FR = 0; PACKET_FR_COMPLETED = false;
                                for (int i = INDEX_FR; i < LENGTH_INT_FR; i++) FOOT_PACKET_RX[i] = (byte)SP1.ReadByte();
                                INDEX_FR = LENGTH_INT_FR; PACKET_FR_COMPLETED = false;
                            }
                            else { ERROR_RF8++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
                        }
                    }
                }
            }
            catch { ERROR_RF9++; INDEX_FR = 0; LENGTH_INT_FR = 0; }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void PACKET_ASSESSMENT_LF()
        {
            if (((char)FOOT_PACKET_LX[0] == 'L') && ((char)FOOT_PACKET_LX[1] == 'T') && ((char)FOOT_PACKET_LX[2] == 'F'))
            {
                countMicroFL = FOOT_PACKET_LX[3];
                countMicroLF[0] = countMicroFL;
                if (!Calibration)
                {
                    if (((char)FOOT_PACKET_LX[22] == 'S') && ((char)FOOT_PACKET_LX[23] == 'T'))
                    {
                        if (FOOT_PACKET_LX[3] == FOOT_PACKET_LX[24])
                        {
                            if (DataLossControlLF)
                            {
                                if ((countMicroLF[0] - countMicroLF[1] > 1) && (countMicroLF[0] > countMicroLF[1]))
                                {
                                    countMicroLossLF = countMicroLossLF + (countMicroLF[0] - countMicroLF[1]) - 1;
                                }
                                else if (countMicroLF[1] > countMicroLF[0])
                                {
                                    countMicroLossLF = countMicroLossLF + (255 - countMicroLF[1] + countMicroLF[0]);
                                }
                            }
                            frequencyLF++; countfreqLF++;
                            countMicroLF[1] = countMicroLF[0];
                            accxF_LX = ((float)((Int16)((FOOT_PACKET_LX[5] << 8) + FOOT_PACKET_LX[4]))) / 100;
                            accyF_LX = ((float)((Int16)((FOOT_PACKET_LX[7] << 8) + FOOT_PACKET_LX[6]))) / 100;
                            acczF_LX = ((float)((Int16)((FOOT_PACKET_LX[9] << 8) + FOOT_PACKET_LX[8]))) / 100;
                            gyrxF_LX = ((float)((Int16)((FOOT_PACKET_LX[11] << 8) + FOOT_PACKET_LX[10]))) / 10;
                            gyryF_LX = ((float)((Int16)((FOOT_PACKET_LX[13] << 8) + FOOT_PACKET_LX[12]))) / 10;
                            gyrzF_LX = ((float)((Int16)((FOOT_PACKET_LX[15] << 8) + FOOT_PACKET_LX[14]))) / 10;
                            if ((DATA_LOG) || (DATA_GRAPH))
                            {
                                magxF_LX = ((float)((Int16)((FOOT_PACKET_LX[17] << 8) + FOOT_PACKET_LX[16]))) / 100;
                                magyF_LX = ((float)((Int16)((FOOT_PACKET_LX[19] << 8) + FOOT_PACKET_LX[18]))) / 100;
                                magzF_LX = ((float)((Int16)((FOOT_PACKET_LX[21] << 8) + FOOT_PACKET_LX[20]))) / 100;
                            }
                            else if (DATA_AHRS)
                            {
                                rollF_LX = ((float)((Int16)((FOOT_PACKET_LX[17] << 8) + FOOT_PACKET_LX[16]))) / 100;
                                pitchF_LX = ((float)((Int16)((FOOT_PACKET_LX[19] << 8) + FOOT_PACKET_LX[18]))) / 100;
                                yawF_LX = ((float)((Int16)((FOOT_PACKET_LX[21] << 8) + FOOT_PACKET_LX[20]))) / 100;
                            }

                            UpdatePanel(workMode);

                            if (path_existFLX)
                            {
                                accxF_LX_txt = String.Format("{0:000.0000}", accxF_LX).ToString(); accyF_LX_txt = String.Format("{0:000.0000}", accyF_LX).ToString(); acczF_LX_txt = String.Format("{0:000.0000}", acczF_LX).ToString();
                                gyrxF_LX_txt = String.Format("{0:000.0000}", gyrxF_LX).ToString(); gyryF_LX_txt = String.Format("{0:000.0000}", gyryF_LX).ToString(); gyrzF_LX_txt = String.Format("{0:000.0000}", gyrzF_LX).ToString();
                                StreamWriter FileDATA_LF = File.AppendText(pathFLX);
                                FileDATA_LF.Flush();
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magxF_LX_txt = String.Format("{0:000.0000}", magxF_LX).ToString(); magyF_LX_txt = String.Format("{0:000.0000}", magyF_LX).ToString(); magzF_LX_txt = String.Format("{0:000.0000}", magzF_LX).ToString();
                                    report = countfreqLF.ToString() + "\t" + "\t" + accxF_LX_txt + "\t" + accyF_LX_txt + "\t" + acczF_LX_txt + "\t" + gyrxF_LX_txt + "\t" + gyryF_LX_txt + "\t" + gyrzF_LX_txt + "\t" + magxF_LX_txt + "\t"
                                        + magyF_LX_txt + "\t" + magzF_LX_txt + "\t" + countMicroFL + "\r\n";
                                }
                                else if (DATA_AHRS)
                                {
                                    rollF_LX_txt = String.Format("{0:000.0000}", rollF_LX).ToString(); pitchF_LX_txt = String.Format("{0:000.0000}", pitchF_LX).ToString(); yawF_LX_txt = String.Format("{0:000.0000}", yawF_LX).ToString();
                                    report = countfreqLF.ToString() + "\t" + "\t" + accxF_LX_txt + "\t" + accyF_LX_txt + "\t" + acczF_LX_txt + "\t" + gyrxF_LX_txt + "\t" + gyryF_LX_txt + "\t" + gyrzF_LX_txt + "\t" + rollF_LX_txt + "\t"
                                        + pitchF_LX_txt + "\t" + yawF_LX_txt + "\t" + countMicroFL + "\r\n";
                                }

                                FileDATA_LF.WriteLine(report); FileDATA_LF.Flush(); FileDATA_LF.Close();
                            }
                            PACKET_FL_COMPLETED = true;
                        }
                    }
                }
            }
        }
        /*---------------------------------------------------------------------------------------------------------------*/
        void PACKET_ASSESSMENT_RF()
        {
            if (((char)FOOT_PACKET_RX[0] == 'R') && ((char)FOOT_PACKET_RX[1] == 'H') && ((char)FOOT_PACKET_RX[2] == 'F'))
            {
                countMicroFR = FOOT_PACKET_RX[3];
                countMicroRF[0] = countMicroFR;
                if (!Calibration)
                {
                    if (((char)FOOT_PACKET_RX[22] == 'S') && ((char)FOOT_PACKET_RX[23] == 'R'))
                    {
                        if (FOOT_PACKET_RX[3] == FOOT_PACKET_RX[24])
                        {
                            if (DataLossControlRF)
                            {
                                if ((countMicroRF[0] - countMicroRF[1] > 1) && (countMicroRF[0] > countMicroRF[1]))
                                {
                                    countMicroLossRF = countMicroLossRF + (countMicroRF[0] - countMicroRF[1]) - 1;
                                }
                                else if (countMicroRF[1] > countMicroRF[0])
                                {
                                    countMicroLossRF = countMicroLossRF + (255 - countMicroRF[1] + countMicroRF[0]);
                                }
                            }
                            frequencyRF++; countfreqRF++;
                            countMicroRF[1] = countMicroRF[0];
                            accxF_RX = ((float)((Int16)((FOOT_PACKET_RX[5] << 8) + FOOT_PACKET_RX[4]))) / 100;
                            accyF_RX = ((float)((Int16)((FOOT_PACKET_RX[7] << 8) + FOOT_PACKET_RX[6]))) / 100;
                            acczF_RX = ((float)((Int16)((FOOT_PACKET_RX[9] << 8) + FOOT_PACKET_RX[8]))) / 100;
                            gyrxF_RX = ((float)((Int16)((FOOT_PACKET_RX[11] << 8) + FOOT_PACKET_RX[10]))) / 10;
                            gyryF_RX = ((float)((Int16)((FOOT_PACKET_RX[13] << 8) + FOOT_PACKET_RX[12]))) / 10;
                            gyrzF_RX = ((float)((Int16)((FOOT_PACKET_RX[15] << 8) + FOOT_PACKET_RX[14]))) / 10;
                            if ((DATA_LOG) || (DATA_GRAPH))
                            {
                                magxF_RX = ((float)((Int16)((FOOT_PACKET_RX[17] << 8) + FOOT_PACKET_RX[16]))) / 100;
                                magyF_RX = ((float)((Int16)((FOOT_PACKET_RX[19] << 8) + FOOT_PACKET_RX[18]))) / 100;
                                magzF_RX = ((float)((Int16)((FOOT_PACKET_RX[21] << 8) + FOOT_PACKET_RX[20]))) / 100;
                            }
                            else if (DATA_AHRS)
                            {
                                rollF_RX = ((float)((Int16)((FOOT_PACKET_RX[17] << 8) + FOOT_PACKET_RX[16]))) / 100;
                                pitchF_RX = ((float)((Int16)((FOOT_PACKET_RX[19] << 8) + FOOT_PACKET_RX[18]))) / 100;
                                yawF_RX = ((float)((Int16)((FOOT_PACKET_RX[21] << 8) + FOOT_PACKET_RX[20]))) / 100;
                            }

                            UpdatePanel(workMode);

                            if (path_existFRX)
                            {
                                accxF_RX_txt = String.Format("{0:000.0000}", accxF_RX).ToString(); accyF_RX_txt = String.Format("{0:000.0000}", accyF_RX).ToString(); acczF_RX_txt = String.Format("{0:000.0000}", acczF_RX).ToString();
                                gyrxF_RX_txt = String.Format("{0:000.0000}", gyrxF_RX).ToString(); gyryF_RX_txt = String.Format("{0:000.0000}", gyryF_RX).ToString(); gyrzF_RX_txt = String.Format("{0:000.0000}", gyrzF_RX).ToString();
                                StreamWriter FileDATA_RF = File.AppendText(pathFRX);
                                FileDATA_RF.Flush();
                                if ((DATA_LOG) || (DATA_GRAPH))
                                {
                                    magxF_RX_txt = String.Format("{0:000.0000}", magxF_RX).ToString(); magyF_RX_txt = String.Format("{0:000.0000}", magyF_RX).ToString(); magzF_RX_txt = String.Format("{0:000.0000}", magzF_RX).ToString();
                                    report = countfreqRF.ToString() + "\t" + "\t" + accxF_RX_txt + "\t" + accyF_RX_txt + "\t" + acczF_RX_txt + "\t" + gyrxF_RX_txt + "\t" + gyryF_RX_txt + "\t" + gyrzF_RX_txt + "\t" + magxF_RX_txt + "\t"
                                        + magyF_RX_txt + "\t" + magzF_RX_txt + "\t" + countMicroFR + "\r\n";
                                }
                                else if (DATA_AHRS)
                                {
                                    rollF_RX_txt = String.Format("{0:000.0000}", rollF_RX).ToString(); pitchF_RX_txt = String.Format("{0:000.0000}", pitchF_RX).ToString(); yawF_RX_txt = String.Format("{0:000.0000}", yawF_RX).ToString();
                                    report = countfreqRF.ToString() + "\t" + "\t" + accxF_RX_txt + "\t" + accyF_RX_txt + "\t" + acczF_RX_txt + "\t" + gyrxF_RX_txt + "\t" + gyryF_RX_txt + "\t" + gyrzF_RX_txt + "\t" + rollF_RX_txt + "\t"
                                        + pitchF_RX_txt + "\t" + yawF_RX_txt + "\t" + countMicroFR + "\r\n";
                                }

                                FileDATA_RF.WriteLine(report); FileDATA_RF.Flush(); FileDATA_RF.Close();
                            }
                            PACKET_FR_COMPLETED = true;
                        }
                    }
                }
            }
        }

        /*---------------------------------------------------------------------------------------------------------------*/
        private void TXT_SAVE()
        {
            var date = DateTime.Now;
            if (!Calibration)
            {
                int count_testHLX = 1;
                int count_testFLX = 1;
                int count_testHRX = 1;
                int count_testFRX = 1;
                if ((count_testHLX < 10) && ((working_mode == 0) || (working_mode == 4) || (working_mode == 6))) FileNameHLX = "HAND_LX" + "_0" + count_testHLX.ToString() + ".txt";
                else if ((count_testHLX >= 10) && ((working_mode == 0) || (working_mode == 4) || (working_mode == 6))) FileNameHLX = "HAND_LX" + "_" + count_testHLX.ToString() + ".txt";
                if ((count_testHRX < 10) && ((working_mode == 1) || (working_mode == 4) || (working_mode == 7))) FileNameHRX = "HAND_RX" + "_0" + count_testHRX.ToString() + ".txt";
                else if ((count_testHRX >= 10) && ((working_mode == 1) || (working_mode == 4) || (working_mode == 7))) FileNameHRX = "HAND_RX" + "_" + count_testHRX.ToString() + ".txt";
                if ((count_testFLX < 10) && ((working_mode == 2) || (working_mode == 5) || (working_mode == 6))) FileNameFLX = "FOOT_LX" + "_0" + count_testFLX.ToString() + ".txt";
                else if ((count_testFLX >= 10) && ((working_mode == 2) || (working_mode == 5) || (working_mode == 6))) FileNameFLX = "FOOT_LX" + "_" + count_testFLX.ToString() + ".txt";
                if ((count_testFRX < 10) && ((working_mode == 3) || (working_mode == 5) || (working_mode == 7))) FileNameFRX = "FOOT_RX" + "_0" + count_testFRX.ToString() + ".txt";
                else if ((count_testFRX >= 10) && ((working_mode == 3) || (working_mode == 5) || (working_mode == 7))) FileNameFRX = "FOOT_RX" + "_" + count_testFRX.ToString() + ".txt";
                if ((working_mode == 0) || (working_mode == 4) || (working_mode == 6)) pathHLX = System.IO.Path.Combine(starting_pathTXT, FileNameHLX);
                if ((working_mode == 2) || (working_mode == 5) || (working_mode == 6)) pathFLX = System.IO.Path.Combine(starting_pathTXT, FileNameFLX);
                if ((working_mode == 1) || (working_mode == 4) || (working_mode == 7)) pathHRX = System.IO.Path.Combine(starting_pathTXT, FileNameHRX);
                if ((working_mode == 3) || (working_mode == 5) || (working_mode == 7)) pathFRX = System.IO.Path.Combine(starting_pathTXT, FileNameFRX);
                while ((File.Exists(pathHLX)) && ((working_mode == 0) || (working_mode == 4) || (working_mode == 6)))
                {
                    count_testHLX++;
                    if (count_testHLX < 10) FileNameHLX = "HAND_LX" + "_0" + count_testHLX.ToString() + ".txt";
                    else FileNameHLX = "HAND_LX" + "_" + count_testHLX.ToString() + ".txt";

                    pathHLX = System.IO.Path.Combine(starting_pathTXT, FileNameHLX);
                }
                while ((File.Exists(pathHRX)) && ((working_mode == 1) || (working_mode == 4) || (working_mode == 7)))
                {
                    count_testHRX++;
                    if (count_testHRX < 10) FileNameHRX = "HAND_RX" + "_0" + count_testHRX.ToString() + ".txt";
                    else FileNameHRX = "HAND_RX" + "_" + count_testHRX.ToString() + ".txt";

                    pathHRX = System.IO.Path.Combine(starting_pathTXT, FileNameHRX);
                }
                while ((File.Exists(pathFLX)) && ((working_mode == 2) || (working_mode == 5) || (working_mode == 6)))
                {
                    count_testFLX++;
                    if (count_testFLX < 10) FileNameFLX = "FOOT_LX" + "_0" + count_testFLX.ToString() + ".txt";
                    else FileNameFLX = "FOOT_LX" + "_" + count_testFLX.ToString() + ".txt";

                    pathFLX = System.IO.Path.Combine(starting_pathTXT, FileNameFLX);
                }
                while ((File.Exists(pathFRX)) && ((working_mode == 3) || (working_mode == 5) || (working_mode == 7)))
                {
                    count_testFRX++;
                    if (count_testFRX < 10) FileNameFRX = "FOOT_RX" + "_0" + count_testFRX.ToString() + ".txt";
                    else FileNameFRX = "FOOT_RX" + "_" + count_testFRX.ToString() + ".txt";

                    pathFRX = System.IO.Path.Combine(starting_pathTXT, FileNameFRX);
                }
                try
                {
                    if ((working_mode == 0) || (working_mode == 4) || (working_mode == 6))
                    {
                        StreamWriter Nome_fileHLX = new StreamWriter(pathHLX);
                        Nome_fileHLX.Close();
                        path_existHLX = true;
                        StreamWriter fileDatiHLX = File.AppendText(pathHLX);
                        fileDatiHLX.Flush();
                        if ((DATA_LOG) || (DATA_GRAPH))
                        {
                            report = "Test number: " + count_testHLX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + " " + "AccW_x" + "\t" + "\t" + "AccW_y" + "\t" + "\t" + "AccW_z" + "\t" + "\t" + "GirW_x" + "\t" + "\t" + "GirW_y" + "\t" + "\t" + "GirW_z"
                                    + "\t" + "\t" + "MagW_x" + "\t" + "\t" + "MagW_y" + "\t" + "\t" + "MagW_z" + "\t" + "\t" + "AccT_x" + "\t" + "\t" + "AccT_y" + "\t" + "\t" + "AccT_z" + "\t" + "\t" + "GirT_x" + "\t" + "\t" + "GirT_y" + "\t" + "\t" + "GirT_z" + "\t" + "\t" + "MagT_x" + "\t" + "\t" + "MagT_y" + "\t" + "\t" + "MagT_z"
                                    + "\t" + "\t" + "AccI_x" + "\t" + "\t" + "AccI_y" + "\t" + "\t" + "AccI_z" + "\t" + "\t" + "GirI_x" + "\t" + "\t" + "GirI_y" + "\t" + "\t" + "GirI_z" + "\t" + "\t" + "MagI_x" + "\t" + "\t" + "MagI_y" + "\t" + "\t" + "MagI_z"
                                    + "\t" + "\t" + "AccM_x" + "\t" + "\t" + "AccM_y" + "\t" + "\t" + "AccM_z" + "\t" + "\t" + "GirM_x" + "\t" + "\t" + "GirM_y" + "\t" + "\t" + "GirM_z" + "\t" + "\t" + "MagM_x" + "\t" + "\t" + "MagM_y" + "\t" + "\t" + "MagM_z"
                                    + "\t" + "\t" + "CounterMicro";
                        }
                        else if (DATA_AHRS)
                        {
                            report = "Test number: " + count_testHLX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + " " + "AccW_x" + "\t" + "\t" + "AccW_y" + "\t" + "\t" + "AccW_z" + "\t" + "\t" + "GirW_x" + "\t" + "\t" + "GirW_y" + "\t" + "\t" + "GirW_z"
                                    + "\t" + "\t" + "RollW" + "\t" + "\t" + "PitchW" + "\t" + "\t" + "YawW" + "\t" + "\t" + "AccT_x" + "\t" + "\t" + "AccT_y" + "\t" + "\t" + "AccT_z" + "\t" + "\t" + "GirT_x" + "\t" + "\t" + "GirT_y" + "\t" + "\t" + "GirT_z" + "\t" + "\t" + "RollT" + "\t" + "\t" + "PitchT" + "\t" + "\t" + "YawT"
                                    + "\t" + "\t" + "AccI_x" + "\t" + "\t" + "AccI_y" + "\t" + "\t" + "AccI_z" + "\t" + "\t" + "GirI_x" + "\t" + "\t" + "GirI_y" + "\t" + "\t" + "GirI_z" + "\t" + "\t" + "RollI" + "\t" + "\t" + "PitchI" + "\t" + "\t" + "YawI"
                                    + "\t" + "\t" + "AccM_x" + "\t" + "\t" + "AccM_y" + "\t" + "\t" + "AccM_z" + "\t" + "\t" + "GirM_x" + "\t" + "\t" + "GirM_y" + "\t" + "\t" + "GirM_z" + "\t" + "\t" + "RollM" + "\t" + "\t" + "PitchM" + "\t" + "\t" + "YawM"
                                    + "\t" + "\t" + "CounterMicro";
                        }
                        fileDatiHLX.WriteLine(report);
                        fileDatiHLX.Flush();
                        fileDatiHLX.Close();
                    }
                    if ((working_mode == 1) || (working_mode == 4) || (working_mode == 7))
                    {
                        StreamWriter Nome_fileHRX = new StreamWriter(pathHRX);
                        Nome_fileHRX.Close();
                        path_existHRX = true;
                        StreamWriter fileDatiHRX = File.AppendText(pathHRX);
                        fileDatiHRX.Flush();
                        if ((DATA_LOG) || (DATA_GRAPH))
                        {
                            report = "Test number: " + count_testHRX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + " " + "AccW_x" + "\t" + "\t" + "AccW_y" + "\t" + "\t" + "AccW_z" + "\t" + "\t" + "GirW_x" + "\t" + "\t" + "GirW_y" + "\t" + "\t" + "GirW_z"
                                    + "\t" + "\t" + "MagW_x" + "\t" + "\t" + "MagW_y" + "\t" + "\t" + "MagW_z" + "\t" + "\t" + "AccT_x" + "\t" + "\t" + "AccT_y" + "\t" + "\t" + "AccT_z" + "\t" + "\t" + "GirT_x" + "\t" + "\t" + "GirT_y" + "\t" + "\t" + "GirT_z" + "\t" + "\t" + "MagT_x" + "\t" + "\t" + "MagT_y" + "\t" + "\t" + "MagT_z"
                                    + "\t" + "\t" + "AccI_x" + "\t" + "\t" + "AccI_y" + "\t" + "\t" + "AccI_z" + "\t" + "\t" + "GirI_x" + "\t" + "\t" + "GirI_y" + "\t" + "\t" + "GirI_z" + "\t" + "\t" + "MagI_x" + "\t" + "\t" + "MagI_y" + "\t" + "\t" + "MagI_z"
                                    + "\t" + "\t" + "AccM_x" + "\t" + "\t" + "AccM_y" + "\t" + "\t" + "AccM_z" + "\t" + "\t" + "GirM_x" + "\t" + "\t" + "GirM_y" + "\t" + "\t" + "GirM_z" + "\t" + "\t" + "MagM_x" + "\t" + "\t" + "MagM_y" + "\t" + "\t" + "MagM_z"
                                    + "\t" + "\t" + "CounterMicro";
                        }
                        else if (DATA_AHRS)
                        {
                            report = "Test number: " + count_testHRX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + " " + "AccW_x" + "\t" + "\t" + "AccW_y" + "\t" + "\t" + "AccW_z" + "\t" + "\t" + "GirW_x" + "\t" + "\t" + "GirW_y" + "\t" + "\t" + "GirW_z"
                                    + "\t" + "\t" + "RollW" + "\t" + "\t" + "PitchW" + "\t" + "\t" + "YawW" + "\t" + "\t" + "AccT_x" + "\t" + "\t" + "AccT_y" + "\t" + "\t" + "AccT_z" + "\t" + "\t" + "GirT_x" + "\t" + "\t" + "GirT_y" + "\t" + "\t" + "GirT_z" + "\t" + "\t" + "RollT" + "\t" + "\t" + "PitchT" + "\t" + "\t" + "YawT"
                                    + "\t" + "\t" + "AccI_x" + "\t" + "\t" + "AccI_y" + "\t" + "\t" + "AccI_z" + "\t" + "\t" + "GirI_x" + "\t" + "\t" + "GirI_y" + "\t" + "\t" + "GirI_z" + "\t" + "\t" + "RollI" + "\t" + "\t" + "PitchI" + "\t" + "\t" + "YawI"
                                    + "\t" + "\t" + "AccM_x" + "\t" + "\t" + "AccM_y" + "\t" + "\t" + "AccM_z" + "\t" + "\t" + "GirM_x" + "\t" + "\t" + "GirM_y" + "\t" + "\t" + "GirM_z" + "\t" + "\t" + "RollM" + "\t" + "\t" + "PitchM" + "\t" + "\t" + "YawM"
                                    + "\t" + "\t" + "CounterMicro";
                        }
                        fileDatiHRX.WriteLine(report);
                        fileDatiHRX.Flush();
                        fileDatiHRX.Close();
                    }
                    if ((working_mode == 2) || (working_mode == 5) || (working_mode == 6))
                    {
                        StreamWriter Nome_fileFLX = new StreamWriter(pathFLX);
                        Nome_fileFLX.Close();
                        path_existFLX = true;
                        StreamWriter fileDatiFLX = File.AppendText(pathFLX);
                        fileDatiFLX.Flush();
                        if ((DATA_LOG) || (DATA_GRAPH))
                        {
                            report = "Test number: " + count_testFLX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + "\t" + "\t" + "AccF_x" + "\t" + "\t" + "AccF_y" + "\t" + "\t" + "AccF_z" + "\t" + "\t" + "GirF_x" + "\t" + "\t" + "GirF_y" + "\t" + "\t" + "GirF_z"
                                    + "\t" + "\t" + "MagF_x" + "\t" + "\t" + "MagF_y" + "\t" + "\t" + "MagF_z" + "\t" + "\t" + "CounterMicro";
                        }
                        else if (DATA_AHRS)
                        {
                            report = "Test number: " + count_testHLX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + "\t" + "\t" + "AccF_x" + "\t" + "\t" + "AccF_y" + "\t" + "\t" + "AccF_z" + "\t" + "\t" + "GirF_x" + "\t" + "\t" + "GirF_y" + "\t" + "\t" + "GirF_z"
                                    + "\t" + "\t" + "RollF" + "\t" + "\t" + "PitchF" + "\t" + "\t" + "YawF" + "\t" + "\t" + "CounterMicro";
                        }
                        fileDatiFLX.WriteLine(report);
                        fileDatiFLX.Flush();
                        fileDatiFLX.Close();
                    }
                    if ((working_mode == 3) || (working_mode == 5) || (working_mode == 7))
                    {
                        StreamWriter Nome_fileFRX = new StreamWriter(pathFRX);
                        Nome_fileFRX.Close();
                        path_existFRX = true;
                        StreamWriter fileDatiFRX = File.AppendText(pathFRX);
                        fileDatiFRX.Flush();
                        if ((DATA_LOG) || (DATA_GRAPH))
                        {
                            report = "Test number: " + count_testFRX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + "\t" + "\t" + "AccF_x" + "\t" + "\t" + "AccF_y" + "\t" + "\t" + "AccF_z" + "\t" + "\t" + "GirF_x" + "\t" + "\t" + "GirF_y" + "\t" + "\t" + "GirF_z"
                                    + "\t" + "\t" + "MagF_x" + "\t" + "\t" + "MagF_y" + "\t" + "\t" + "MagF_z" + "\t" + "\t" + "CounterMicro";
                        }
                        else if (DATA_AHRS)
                        {
                            report = "Test number: " + count_testHRX + "\r\n" +
                                    "Date:" + date.Day + "/" + date.Month + "/" + date.Year + "\r\n" +
                                    "Time :" + date.Hour + ":" + date.Minute + ":" + date.Second + "\r\n" +
                                    "Counter" + "\t" + "\t" + "AccF_x" + "\t" + "\t" + "AccF_y" + "\t" + "\t" + "AccF_z" + "\t" + "\t" + "GirF_x" + "\t" + "\t" + "GirF_y" + "\t" + "\t" + "GirF_z"
                                    + "\t" + "\t" + "RollF" + "\t" + "\t" + "PitchF" + "\t" + "\t" + "YawF" + "\t" + "\t" + "CounterMicro";
                        }
                        fileDatiFRX.WriteLine(report);
                        fileDatiFRX.Flush();
                        fileDatiFRX.Close();
                    }
                }
                catch { }
            }
        }


        //There you can see in which order shou functions be. You need to sleep thred becouse it take some time Connect() 
        //function to connect.
        public void main()
        {
            FormLoad();
            PickWorkmode();
            ConnectSensor(workMode);
            PickAplication();
            PickMeter();
            Thread.Sleep(4000);
            Rec(workMode);
        }

    }
}