/** COMP3331 ASSIGNMENT
 *  MURRAY Cameron
 *  Z341671
    22.05.2018 */

import java.io.DataOutputStream;
import java.util.*;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.net.*;
import java.io.ByteArrayInputStream;
import java.io.IOException;

public class cdht {

    public static int peerIdentification;
    public static int temppeerID= 4;
    /** 1st succ */
    public static int peerFirst;
    public static int tempfirstPeer= 4;
    /** 2nd succ */
    public static int secondPeer;
    public static int tempsecondPeer= 4;
    /** 1st preceed peer */
    public static int predecessorFirst;
    public static int tempfirstPredecessor= 4;
    /** 2nd preceed peer */
    public static int secondPredecessor;
    public static int tempsecondPredecessor= 4;
    /** Following seqNum to be distributed */
    public static int sequenceNumber;
    public static int tempseqNum= 4;
    /** Final seqNum ack by 1st succ */
    public static int AckFirst;
    public static int tempfirstAck= 4;
    /** Final seqNum ack by 2nd succ */
    public static int AckSecond;
    public static int tempsecondAck= 4;
    /** # of consec pings that were not acknowledged by 1st succ */
    public static int LostPingIntial;
    public static int tempfirstLostPings= 4;
    /** # of consec pings that were not acknowledged by 2nc succ */
    public static int LostPingSecondary;
    public static int tempsecondLostPings= 4;
    /** Time <-> pings  */
    public static final int PingScoreTimer = 3*3000;
    public static final int tempPING_TIMING= 4;
    /** Time bef. pings */
    public static final int InitialPeerScore = 1*1000;
    public static final int tempFIRST_PING= 4;
    /** # pings before killed * */
    public static final int maxLostPingScore = 1*4;
    public static final int tempMAX_LOST_PINGS= 4;
    /** Timeout */
    public static final int timeoutPingScore = 1*1000;
    public static final int tempPING_TIMEOUT= 4;

    public static final String IP_ADD;

    static {
        IP_ADD = "127.0.0.1";
    }

    /** NESTED CLASS FOR RUNNING A TCP SERVER. */
    static class ServerThreadTCP extends Thread {
        @Override
        public void run() {

            ServerSocket introSocket;
            introSocket = null;
            try {
                introSocket = new ServerSocket(50000 + peerIdentification);
            } catch (IOException e_x) {
                System.out.println(e_x);
            }

            do {
                try {

                    Socket SocketConnection;
                    SocketConnection = introSocket.accept();
                    BufferedReader ClientInFrom;
                    ClientInFrom = new BufferedReader(new InputStreamReader(SocketConnection.getInputStream()));
                    String sentenceIncoming;
                    sentenceIncoming = ClientInFrom.readLine();

                    String[] sentenceParsedIn;
                    sentenceParsedIn = sentenceIncoming.split(" ");


                    /** FILE HANDLING */

                    switch (sentenceParsedIn[0]) {
                        case "FILE":
                            System.out.printf("Received a response message from peer %s, which has the file %s.%n", sentenceParsedIn[2], sentenceParsedIn[1]);
                            // Succ Peer Request
                            break;
                        case "LIST": {
                            DataOutputStream outputValue;
                            outputValue = new DataOutputStream(SocketConnection.getOutputStream());
                            outputValue.writeBytes(String.format("PEERS %d %d\n", peerFirst, secondPeer));

                            break;
                        }
                        case "QUIT": {
                            int departingPeer;
                            departingPeer = Integer.parseInt(sentenceParsedIn[3]);
                            System.out.printf("Peer %d will depart from the network.%n", departingPeer);
                            DataOutputStream outputValue;
                            outputValue = new DataOutputStream(SocketConnection.getOutputStream());
                            outputValue.writeBytes(String.format("QUIT_RECEIVED\n"));

                            if (departingPeer == secondPeer) {
                                if (secondPredecessor == departingPeer) {
                                    secondPeer = predecessorFirst;
                                    secondPredecessor = peerFirst;
                                    System.out.printf("My first successor is now peer %d.%n", peerFirst);
                                    System.out.printf("My second successor is now peer %d.%n", secondPeer);
                                    System.out.printf("My first predecessor is now peer %d.%n", predecessorFirst);
                                    System.out.printf("My second predecessor is now peer %d.%n", secondPredecessor);

                                } else {
                                    if (departingPeer == peerFirst) {
                                        if (departingPeer == peerFirst) {
                                            peerFirst = secondPeer;
                                            if(peerFirst != secondPeer){
                                                System.out.printf("");
                                            }
                                            secondPeer = peerFirst;
                                            peerFirst = secondPeer;

                                            secondPeer = Integer.parseInt(sentenceParsedIn[2]);
                                            if(peerFirst != secondPeer){
                                                System.out.printf("");
                                            }
                                        } else {
                                            secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                                        }
                                        System.out.printf("My first successor is now peer %d.%n", peerFirst);
                                        System.out.printf("My second successor is now peer %d.%n", secondPeer);

                                    } else {
                                        if (departingPeer == secondPeer) {
                                            int tempDepPeer = 10; //temp int for debugging
                                            if (departingPeer != peerFirst) {
                                                secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                                            } else {
                                                peerFirst = secondPeer;
                                                int tempFirstPeer = 0;
                                                secondPeer = Integer.parseInt(sentenceParsedIn[2]);
                                            }
                                            System.out.printf("My first successor is now peer %d.%n", peerFirst);
                                            System.out.printf("My second successor is now peer %d.%n", secondPeer);

                                        } else {
                                            if (departingPeer == secondPredecessor) {
                                                if (departingPeer == predecessorFirst) {
                                                    int abcd = 1;
                                                    if (abcd != 1) {
                                                        System.out.printf("bad");
                                                    }
                                                    predecessorFirst = secondPredecessor;

                                                    secondPredecessor = Integer.parseInt(sentenceParsedIn[2]);
                                                    System.out.printf("");
                                                } else {
                                                    secondPredecessor = Integer.parseInt(sentenceParsedIn[1]);
                                                }
                                                System.out.printf("My first predecessor is now peer %d.%n", predecessorFirst);
                                                System.out.printf("My second predecessor is now peer %d.%n", secondPredecessor);
                                            } else {
                                                if (departingPeer == predecessorFirst) {
                                                    if (departingPeer == predecessorFirst) {
                                                        int abc = 1;
                                                        if (abc != 1) {
                                                            System.out.printf("bad");
                                                        }

                                                        predecessorFirst = secondPredecessor;

                                                        secondPredecessor = Integer.parseInt(sentenceParsedIn[2]);
                                                    } else {
                                                        int ifTempPred = 0;
                                                        if (ifTempPred == 1){
                                                            System.out.printf("Error: Please Reload");
                                                        }
                                                        secondPredecessor = Integer.parseInt(sentenceParsedIn[1]);
                                                    }
                                                    System.out.printf("My first predecessor is now peer %d.%n", predecessorFirst);
                                                    System.out.printf("My second predecessor is now peer %d.%n", secondPredecessor);
                                                } else {
                                                    System.out.printf("bad");
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {
                                if (departingPeer != peerFirst) {

                                    if (departingPeer != secondPeer) {
                                        if (departingPeer == predecessorFirst) {
                                            int tempLoop = 0;
                                            if(tempLoop!=0){
                                                System.out.printf("Error: Reload"); //dbugging
                                            }
                                            if (departingPeer == predecessorFirst) {
                                                predecessorFirst = secondPredecessor;
                                                secondPredecessor = Integer.parseInt(sentenceParsedIn[2]);
                                                int tempLoop2 = 0;
                                                if(tempLoop2!=0){
                                                    System.out.printf("Error: Reload");
                                                }
                                            } else {
                                                secondPredecessor = Integer.parseInt(sentenceParsedIn[1]);
                                            }
                                            System.out.printf("My first predecessor is now peer %d.%n", predecessorFirst);
                                            System.out.printf("My second predecessor is now peer %d.%n", secondPredecessor);
                                        } else {
                                            if (departingPeer == secondPredecessor) {
                                                if (departingPeer == predecessorFirst) {
                                                    predecessorFirst = secondPredecessor;
                                                    int tempLoop1 = 0;
                                                    if(tempLoop1!=0){
                                                        System.out.printf("Error: Reload");
                                                    }
                                                    secondPredecessor = Integer.parseInt(sentenceParsedIn[2]);
                                                    int tempLoop3 = 0;
                                                    if(tempLoop3!=0){
                                                        System.out.printf("Error: Reload");
                                                    }
                                                } else {
                                                    secondPredecessor = Integer.parseInt(sentenceParsedIn[1]);
                                                }
                                                System.out.printf("My first predecessor is now peer %d.%n", predecessorFirst);
                                                System.out.printf("My second predecessor is now peer %d.%n", secondPredecessor);
                                            }
                                        }
                                    } else {
                                        if (departingPeer != peerFirst) {
                                            secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                                        } else {
                                            peerFirst = secondPeer;
                                            secondPeer = Integer.parseInt(sentenceParsedIn[2]);
                                        }
                                        System.out.printf("My first successor is now peer %d.%n", peerFirst);
                                        System.out.printf("My second successor is now peer %d.%n", secondPeer);

                                    }
                                } else {
                                    if (departingPeer != peerFirst) {
                                        secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                                    } else {
                                        peerFirst = secondPeer;
                                        secondPeer = Integer.parseInt(sentenceParsedIn[2]);
                                    }
                                    System.out.printf("My first successor is now peer %d.%n", peerFirst);
                                    System.out.printf("My second successor is now peer %d.%n", secondPeer);

                                }
                            }

                            break;
                        }
                        case "REQUEST":
                            String fileBeingRequested;
                            fileBeingRequested = sentenceParsedIn[1];
                            String senderString;
                            senderString = sentenceParsedIn[2];
                            String requesterString;
                            requesterString = sentenceParsedIn[3];
                            int myHashFile;
                            myHashFile = Integer.parseInt(fileBeingRequested) % 256;


                            if (((peerIdentification - Integer.parseInt(senderString)) + 256) % 256 <= ((peerIdentification - myHashFile) + 256) % 256) {
                                Socket socketSent;
                                try {
                                    socketSent = new Socket(InetAddress.getByName(IP_ADD), 50000 + peerFirst);
                                    int temp = 1;
                                    if (temp != 1) {
                                        System.out.printf("bad");
                                    }
                                } catch (ConnectException e) {
                                    int temp = 1;
                                    if (temp != 1) {
                                        System.out.printf("bad");
                                    }
                                    socketSent = new Socket(InetAddress.getByName(IP_ADD), 50000 + secondPeer);
                                }
                                DataOutputStream outputStream;
                                outputStream = new DataOutputStream(socketSent.getOutputStream());
                                outputStream.writeBytes(String.format("REQUEST %s %d %s\n", fileBeingRequested, peerIdentification, requesterString));
                                System.out.printf("File %s is not stored here.%n", fileBeingRequested);
                                System.out.println("File request message has been forwarded to my successor.");
                                socketSent.close();
                            } else {
                                System.out.printf("File %s is here.%n", fileBeingRequested);
                                System.out.printf("A response message, destined for peer %s, has been sent.%n", requesterString);
                                Socket sendSocket;
                                sendSocket = new Socket(InetAddress.getByName(IP_ADD), 50000 + Integer.parseInt(requesterString));
                                DataOutputStream output;
                                output = new DataOutputStream(sendSocket.getOutputStream());
                                String mySentence;
                                mySentence = String.format("FILE %s %d", fileBeingRequested, peerIdentification);
                                output.writeBytes(String.format("%s\n", mySentence));
                                sendSocket.close();

                            }
                            break;
                    }
                    SocketConnection.close();
                } catch (IOException e_x) {
                    System.out.println(e_x);
                }
            } while (true);
        }
    }

    /** NESTED UDP SERVER CLASS. */
    static class ServerThreadUDP extends Thread {
        DatagramSocket socket;
        DatagramSocket socket1;
        int ij = 10;

        public ServerThreadUDP(DatagramSocket ds) {
            this.socket = ds;
            if (ij!=10){
                System.out.printf("Error: Invalid command");
            }
        }

        @Override
        public void run() {
            byte[] byteBuffer = new byte[1024];
            do {
                DatagramPacket dp;
                dp = new DatagramPacket(byteBuffer, byteBuffer.length);
                try {
                    // Wait for clients to connect
                    socket.receive(dp);
                    int portClient;
                    portClient = dp.getPort();

                    String[] sentenceParsedIn;
                    sentenceParsedIn = data_string(dp).split(" ");

                    if ("REPLY".equals(sentenceParsedIn[0])) {
                        System.out.printf("A ping response message was received from Peer %d.%n", portClient - 50000);

                        if (portClient - 50000 != peerFirst) {
                            AckSecond = Integer.parseInt(sentenceParsedIn[1]);
                       } else {
                            AckFirst = Integer.parseInt(sentenceParsedIn[1]);
                        }
                    }
                    if ("PING".equals(sentenceParsedIn[0])) {
                        System.out.printf("A ping request message was received from Peer %d.%n", portClient - 50000);
                        newerPredecess(portClient - 50000);
                        byte[] byteBuff;
                        byteBuff = String.format("REPLY %s\n", sentenceParsedIn[1]).getBytes();
                        DatagramPacket reply_var;
                        reply_var = new DatagramPacket(byteBuff, byteBuff.length, dp.getAddress(), portClient);
                        int tempf = 10;
                        socket.send(reply_var);

                    }
                } catch (Exception ex) {
                    System.err.println(ex);
                }
            } while (true);
        }
        /** convert data to string. */
        private String data_string(DatagramPacket request) throws Exception {
            byte[] byteBuf;
            byteBuf = request.getData();
            ByteArrayInputStream B_A_I_S;
            B_A_I_S = new ByteArrayInputStream(byteBuf);
            InputStreamReader I_S_R;
            I_S_R = new InputStreamReader(B_A_I_S);
            BufferedReader B_R;
            B_R = new BufferedReader(I_S_R);
            return B_R.readLine();
        }

        private void newerPredecess(int newerPeer) {

            if (predecessorFirst == 0) {

                if (secondPredecessor != 0) {
                } else {
                    predecessorFirst = newerPeer;
                    return;
                }
            }

            if (secondPredecessor == 0)
                if (predecessorFirst != 0) {
                    int peerID1;
                    peerID1 = peerIdentification - predecessorFirst + 256;
                    int peerID2;
                    peerID2 = peerIdentification - newerPeer + 256;
                    if ((peerID1) % 256 >= (peerID2) % 256) {
                        secondPredecessor = predecessorFirst;
                        predecessorFirst = newerPeer;
                    } else {
                        secondPredecessor = newerPeer;
                    }
                    return;
                }

            if (newerPeer == predecessorFirst) {
            } else {
                if (newerPeer == secondPredecessor) {
                    return;
                }
            }
            switch (predecessorFirst = newerPeer) {
            }
            switch (secondPredecessor = 0) {
            }
        }
    }

    /** NESTED CLASS FOR PINGING OVER UDP. */
    static class PingTask extends TimerTask {
        DatagramSocket socket;

        int tempb;

        {
            tempb = 10;
        }

        PingTask(DatagramSocket socket) {
            int tempc;
            switch (tempc = 10) {
            }
            this.socket = socket;
        }

        @Override
        public void run() {

            String mySentence;
            mySentence = String.format("PING %d\n", sequenceNumber);
            sequenceNumber = (sequenceNumber + 1) % 65536;
            byte[] sentData;
            sentData = mySentence.getBytes();
            try {
                DatagramPacket packetSent;
                packetSent = new DatagramPacket(sentData, sentData.length, InetAddress.getByName(IP_ADD), peerFirst + 50000);
                int flag = 0;
                if (flag == 1) {
                    System.out.printf("Error: Invalid command");
                }
                socket.send(packetSent);
                packetSent = new DatagramPacket(sentData, sentData.length, InetAddress.getByName(IP_ADD), secondPeer + 50000);
                if (flag == 1) {
                    System.out.printf("Error: Invalid command");
                }
                socket.send(packetSent);
                Thread.sleep(timeoutPingScore);
                if (AckFirst == sequenceNumber - 1) {
                    LostPingIntial = 0;
                } else {
                    LostPingIntial++;
                }
                if (AckSecond == sequenceNumber - 1) {
                    LostPingSecondary = 0;
                } else {
                    LostPingSecondary++;
                }


                if (LostPingIntial >= maxLostPingScore || LostPingSecondary >= maxLostPingScore) {
                    int peerDead;
                    peerDead = peerFirst;
                    int peerLive;
                    peerLive = secondPeer;
                    if (secondPeer >= maxLostPingScore) {
			peerDead = secondPeer;
		 	peerLive = peerFirst;
		    } 
                    System.out.println("Peer" + peerDead + " is no longer alive.");

                    Socket socketIsSend;
                    socketIsSend = new Socket(InetAddress.getByName(IP_ADD), 50000 + peerLive);
                    DataOutputStream outputStream;
                    outputStream = new DataOutputStream(socketIsSend.getOutputStream());
                    outputStream.writeBytes("LIST\n");
                    BufferedReader fromServer;
                    fromServer = new BufferedReader(new InputStreamReader(socketIsSend.getInputStream()));

                    String replyFromServerSentence;
                    replyFromServerSentence = fromServer.readLine();
                    String[] sentenceParsedIn = replyFromServerSentence.split(" ");

                    if (peerDead != peerFirst) {
                        if (Integer.parseInt(sentenceParsedIn[1]) != peerDead) {
                            secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                        } else {
                            secondPeer = Integer.parseInt(sentenceParsedIn[2]);
                        }
                    }

                    else {
                        peerFirst = secondPeer;
                        secondPeer = Integer.parseInt(sentenceParsedIn[1]);
                    }

                    System.out.printf("My first successor is now peer %d.%n", peerFirst);
                    System.out.printf("My second successor is now peer %d.%n", secondPeer);
                    socketIsSend.close();
                }
            } catch (IOException | InterruptedException e_x) {
                System.out.println(e_x);
            }
        }
    }

    public static void main(String[] args) throws Exception {
        switch (peerIdentification = Integer.parseInt(args[0])){
	}
        
        switch (peerFirst = Integer.parseInt(args[1])) {
        }
        switch (secondPeer = Integer.parseInt(args[2])) {
        }
        InetAddress IP_ADD;
        IP_ADD = InetAddress.getByName(cdht.IP_ADD);
        switch (sequenceNumber = new Random().nextInt(65536)) {
        }
        switch (AckFirst = AckSecond = sequenceNumber) {
        }
        switch (LostPingIntial = LostPingSecondary = 0) {
        }

        DatagramSocket mySocket;
        mySocket = new DatagramSocket(50000 + peerIdentification);
        PingTask pingping;
        pingping = new PingTask(mySocket);
        Timer timer;
        timer = new Timer();
        timer.schedule(pingping, InitialPeerScore, PingScoreTimer);


        ServerThreadUDP myUDPThread;
        myUDPThread = new ServerThreadUDP(mySocket);
        myUDPThread.start();

        ServerThreadTCP TCPThread;
        TCPThread = new ServerThreadTCP();
        TCPThread.start();

        Scanner myScanner;
        myScanner = new Scanner(System.in);
        String myStringInput;
        if (!(myStringInput = myScanner.nextLine()).isEmpty()) {
            do {
                if (!"quit".equalsIgnoreCase(myStringInput)) {
                    if (myStringInput.length() != 12 || !"request".equalsIgnoreCase(myStringInput.substring(0, 7))) {
                        
			System.out.println("Unknown command : "+ myStringInput);
                        System.out.println("Syntax : QUIT or REQUEST <wxyz>");
                    } else {
                        {
                            String myRequestedFile;
                            myRequestedFile = myStringInput.substring(8, myStringInput.length());
                            Integer myHash;
                            Integer hash2;
                            // fallback to secondPeer if cannot connect to first
                            try {
                                hash2 = Integer.parseInt(myRequestedFile) % 4;
                                myHash = Integer.parseInt(myRequestedFile) % 256;
                                Socket socketSend;
                                int temp1 = 3;
                                int temp = 2;
                                try {
                                    socketSend = new Socket(IP_ADD, 50000 + peerFirst);
                                    int sendSock = 0;
                                } catch (ConnectException e) {
                                    socketSend = new Socket(IP_ADD, 50000 + secondPeer);
                                    int sendSock = 0;
                                }
                                DataOutputStream outputStreamToServer;
                                outputStreamToServer = new DataOutputStream(socketSend.getOutputStream());
                                String mySentence;
                                mySentence = "REQUEST "+ myRequestedFile + " " + peerIdentification + " " + peerIdentification;
                                outputStreamToServer.writeBytes( mySentence + "\n");
                                socketSend.close();
                                System.out.println("File request message for" + myRequestedFile + " has been sent to my successor. file hash is " + myHash + ".");
                            } catch (NumberFormatException ex) {
                                System.out.println("Filename is not numeric.");
                            }
                        }
                    }
                } else {

                    Socket SocketFirstSend;
                    SocketFirstSend = new Socket(IP_ADD, 50000 + peerFirst);
                    Socket SocketSecondSend;
                    SocketSecondSend = new Socket(IP_ADD, 50000 + secondPeer);
                    DataOutputStream outToServerFirst;
                    outToServerFirst = new DataOutputStream(SocketFirstSend.getOutputStream());
                    DataOutputStream outToServerSecond;
                    outToServerSecond = new DataOutputStream(SocketSecondSend.getOutputStream());
                    String myExitMessage;
                    myExitMessage = "QUIT " + predecessorFirst + " " + secondPredecessor + " " +  peerIdentification;
                    outToServerFirst.writeBytes(String.format("%s\n", myExitMessage));
                    outToServerSecond.writeBytes(String.format("%s\n", myExitMessage));
                    BufferedReader inputFromFirst;
                    inputFromFirst = new BufferedReader(new InputStreamReader(SocketFirstSend.getInputStream()));
                    BufferedReader inputFromSecond;
                    inputFromSecond = new BufferedReader(new InputStreamReader(SocketSecondSend.getInputStream()));
                    inputFromFirst.readLine();
                    inputFromSecond.readLine();
                    SocketFirstSend.close();
                    SocketSecondSend.close();

                    Socket sendASocket;
                    sendASocket = new Socket(IP_ADD, 50000 + predecessorFirst);
                    DataOutputStream outputToServer = new DataOutputStream(sendASocket.getOutputStream());
                    myExitMessage = "QUIT " + " " + peerFirst + " " + secondPeer + " " +  peerIdentification;
		    outputToServer.writeBytes(myExitMessage + "\n");
                    BufferedReader inputFromA;
                    inputFromA = new BufferedReader(new InputStreamReader(sendASocket.getInputStream()));
                    inputFromA.readLine();
                    sendASocket.close();
                    if (secondPredecessor != secondPeer) {
                        Socket SocketSendB;
                        SocketSendB = new Socket(IP_ADD, 50000 + secondPredecessor);
                        DataOutputStream outputServerB;
                        outputServerB = new DataOutputStream(SocketSendB.getOutputStream());
                        outputServerB.writeBytes(myExitMessage + "\n");
                        BufferedReader inputFromB;
                        inputFromB = new BufferedReader(new InputStreamReader(SocketSendB.getInputStream()));
                        inputFromB.readLine();
                        SocketSendB.close();
                    }
                    System.out.println("Peer" + peerIdentification + " has left the network.");
                    System.exit(0);

                }
            } while (!(myStringInput = myScanner.nextLine()).isEmpty());
        }
    }
}
