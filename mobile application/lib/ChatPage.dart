import 'dart:async'; // For asynchronous programming
import 'dart:convert'; // For JSON and UTF8 encoding/decoding
import 'dart:typed_data'; // For Uint8List data type

import 'package:logger/logger.dart'; // For logging
import './helpers/Message.dart'; // Import custom Message class

import 'package:flutter/foundation.dart'; // For debug mode checks
import 'package:flutter/material.dart'; // For Flutter UI components
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart'; // For Bluetooth functionality

import 'GraphPage.dart'; // Import GraphPage for visualizing data

// Main widget for the Chat Page
class ChatPage extends StatefulWidget {
  final BluetoothDevice server; // Bluetooth device to connect to

  const ChatPage({required this.server}); // Constructor requiring server

  @override
  _ChatPage createState() => new _ChatPage(); // Create state for the widget
}

class _ChatPage extends State<ChatPage> {
  static const clientID = 0; // Client ID for identifying messages
  BluetoothConnection? connection; // Bluetooth connection instance

  List<Message> messages =
      List<Message>.empty(growable: true); // List to hold messages

  // Text editing controller for the input field
  final TextEditingController textEditingController =
      new TextEditingController();
  final ScrollController listScrollController =
      new ScrollController(); // Controller for scrolling

  bool isConnecting = true; // Flag for connection status
  bool get isConnected =>
      (connection?.isConnected ?? false); // Check if connected

  bool isDisconnecting = false; // Flag for disconnection status

  @override
  void initState() {
    super.initState(); // Call superclass method

    // Establish Bluetooth connection
    BluetoothConnection.toAddress(widget.server.address).then((_connection) {
      if (kDebugMode) {
        print('Connected to the device'); // Debug output
      }
      connection = _connection; // Set connection instance
      setState(() {
        isConnecting = false; // Update connection status
        isDisconnecting = false; // Update disconnection status
      });

      // Send a message once the connection is established
      String message = "a"; // Example message to send

      // Ensure connection and output are not null, then send data as bytes
      connection?.output.add(Uint8List.fromList(utf8.encode("$message\r\n")));

      // Listen for incoming data
      connection!.input!.listen(_onDataReceived).onDone(() {
        // Handle disconnection
        if (isDisconnecting) {
          print('Disconnecting locally!'); // Local disconnection
        } else {
          print('Disconnected remotely!'); // Remote disconnection
        }
        if (this.mounted) {
          setState(() {}); // Update state if widget is still mounted
        }
      });
    }).catchError((error) {
      // Handle connection errors
      print('Cannot connect, exception occurred');
      print(error);
    });
  }

  @override
  void dispose() {
    // Clean up resources
    if (isConnected) {
      isDisconnecting = true; // Set disconnection flag
      connection?.dispose(); // Dispose of connection
      connection = null; // Nullify connection
    }

    super.dispose(); // Call superclass method
  }

  @override
  Widget build(BuildContext context) {
    final serverName = widget.server.name ?? "Unknown"; // Get server name

    return Scaffold(
      appBar: AppBar(
        backgroundColor: Colors.deepPurpleAccent, // Set app bar color
        elevation: 8.0, // Adds shadow to the AppBar
        centerTitle: true, // Centers the title
        actions: [
          // Show graph button if connected
          if (isConnected)
            IconButton(
              icon: Icon(Icons.show_chart),
              onPressed: () {
                // Navigate to GraphPage
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (context) => GraphPage(messages: messages),
                  ),
                );
              },
            ),
          // Show Bluetooth status icons
          if (!isConnected && !isConnecting)
            const Icon(Icons.bluetooth_disabled,
                color: Colors.redAccent, size: 28.0),
          if (isConnected)
            const Icon(Icons.bluetooth_connected,
                color: Colors.lightGreenAccent, size: 28.0),
          if (!isConnected && !isConnecting)
            const Icon(Icons.bluetooth_disabled,
                color: Colors.redAccent, size: 28.0),
        ],
        title: AnimatedSwitcher(
          duration: const Duration(milliseconds: 500), // Animation duration
          transitionBuilder: (Widget child, Animation<double> animation) {
            // Transition effect for title
            return FadeTransition(opacity: animation, child: child);
          },
          child: Text(
            isConnecting
                ? 'Connecting to $serverName...'
                : isConnected
                    ? 'Collecting Data from $serverName'
                    : 'Chat Log with $serverName',
            key: ValueKey<String>(isConnecting
                ? 'connecting'
                : isConnected
                    ? 'collecting'
                    : 'chatLog'),
            style: const TextStyle(
              color: Colors.white,
              fontWeight: FontWeight.bold,
              fontSize: 20.0,
            ),
          ),
        ),
        leading: const Icon(Icons.chat_bubble,
            color: Colors.white, size: 28.0), // Leading icon
      ),
      body: SafeArea(
        child: Column(
          children: <Widget>[
            Flexible(
              child: ListView.builder(
                padding: const EdgeInsets.all(12.0), // Padding for the list
                controller: listScrollController, // Scroll controller
                itemCount: messages.length, // Number of messages
                itemBuilder: (BuildContext context, int index) {
                  final Message message = messages[index]; // Get the message
                  List<String> devs =
                      message.text.split(","); // Split message text
                  return Card(
                    elevation: 4, // Card elevation
                    shape: RoundedRectangleBorder(
                      borderRadius:
                          BorderRadius.circular(10), // Rounded corners
                    ),
                    margin: EdgeInsets.symmetric(
                        vertical: 8.0), // Space between cards
                    child: ListTile(
                      leading: Icon(
                        message.whom == clientID
                            ? Icons.person
                            : Icons.bluetooth, // Icon based on message sender
                        size: 40.0,
                        color: message.whom == clientID
                            ? Colors.blueAccent
                            : Colors.grey,
                      ),
                      title: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            _getCurrentDateTime(
                                message.timestamp), // Display timestamp
                            style: const TextStyle(
                              fontSize: 14.0,
                              color: Colors.black54, // Adjust color as needed
                            ),
                          ),
                          const SizedBox(
                              height:
                                  8.0), // Space between timestamp and content
                          devs.length == 1
                              ? Text(
                                  devs[0], // Display message content
                                  style: TextStyle(
                                    fontSize: 16.0,
                                    color: message.whom == clientID
                                        ? Colors.blueAccent
                                        : Colors.grey,
                                  ),
                                )
                              : Row(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(
                                        'X: ${devs.length > 0 ? devs[0] : "N/A"}'),
                                    const SizedBox(width: 16.0),
                                    Text(
                                        'Y: ${devs.length > 1 ? devs[1] : "N/A"}'),
                                    const SizedBox(width: 16.0),
                                    Text(
                                        'Z: ${devs.length > 2 ? devs[2] : "N/A"}'),
                                  ],
                                ),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ),
            Row(
              children: <Widget>[
                Flexible(
                  child: Container(
                    margin: const EdgeInsets.only(left: 16.0),
                    child: TextField(
                      style: const TextStyle(fontSize: 15.0),
                      controller: textEditingController, // Controller for input
                      decoration: InputDecoration.collapsed(
                        hintText: isConnecting
                            ? 'Wait until connected...'
                            : 'Connection established',
                        hintStyle:
                            const TextStyle(color: Colors.grey), // Hint style
                      ),
                      enabled: isConnected, // Enable text field if connected
                    ),
                  ),
                ),
                Container(
                  margin: const EdgeInsets.all(8.0),
                  child: IconButton(
                    icon: const Icon(Icons.send), // Send button
                    onPressed: isConnected
                        ? () => _sendMessage(
                            textEditingController.text) // Send message on press
                        : null,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  final logger = Logger(); // Logger instance for logging messages

  // Data stream for reassembling split data
  List<int> dataStream = [];
  DateTime? timeOffset; // Offset for time calculations
  Duration cycleLength = const Duration(minutes: 15); // Length of the cycle


  // BLUETOOTH SPLITS DATA WHEN ITS TOO LONG. WE IMPLEMENT DATASTREAM TO REASSEMBLE DATA BEFORE CONVERTING IT, IN THAT CASE. WE SEND "\r" AKA "13 in bytes"
  // TO DETERMINE THAT ALL THE DATA HAS BEEN SENT. ONLY THEN, WOULD THE MESSAGES BE CONVERTED. OTHERWISE, IT JUST CONCATINATES IT TO THE PREVIOUS DATASTREAM.
  // DATASTREAM IS RE-INITIALIZED TO [], WHEN THAT'S DONE.
  void _onDataReceived(Uint8List data) {
    // Handle incoming data
    if (data.isNotEmpty && data.last == 35 && timeOffset == null) {
      // If data ends with a specific character, calculate time offset
      String dataString = String.fromCharCodes(data);
      String modifiedDataString = dataString.substring(
          0, dataString.length - 1); // Remove last character
      int remainderAsInt = int.parse(modifiedDataString); // Convert to integer
      timeOffset = DateTime.now().subtract(
          Duration(milliseconds: remainderAsInt)); // Calculate time offset
      return;
    }

    dataStream = dataStream + data; // Append incoming data to the stream

    // If the last byte isn't a specific character, return
    if (data.isNotEmpty && data.last != 13) {
      return;
    }
    String dataString =
        String.fromCharCodes(dataStream); // Convert data stream to string
    print("receivedData: $dataString");
    print(dataString.length);

    // Split the data string at each occurrence of '|'
    List<String> entries = dataString.split("|");

    List<Message> newMessages = []; // List to hold new messages

    // Get maximum index from the last entry
    int maximumInd = int.parse(entries.last.split(",")[3]);
    logger.w(maximumInd); // Log maximum index

    // Iterate through each entry, ignoring any empty entries
    for (String entry in entries) {
      int index = int.parse(entry.split(",")[3]);
      logger.w(maximumInd - index); // Log index difference
      if (entry.isNotEmpty) {
        // Add new message to the list
        newMessages.add(
          Message(
            1,
            entry,
            timeOffset!.subtract(cycleLength *
                (maximumInd - index)), // Calculate message timestamp
          ),
        );
      }
    }

    // Add all new messages at once and trigger a single rebuild
    if (newMessages.isNotEmpty) {
      setState(() {
        messages.addAll(newMessages); // Update messages list
      });
    }
    dataStream = []; // Reset data stream
  }

  void _sendMessage(String text) async {
    text = text.trim(); // Trim whitespace
    textEditingController.clear(); // Clear input field

    if (text.isNotEmpty) {
      try {
        // Send message as bytes
        connection!.output.add(Uint8List.fromList(utf8.encode("$text\r\n")));
        await connection!.output.allSent; // Ensure message is sent

        setState(() {
          messages.add(Message(
              clientID, text, DateTime.now())); // Add sent message to list
        });

        // Scroll to the bottom of the list
        Future.delayed(const Duration(milliseconds: 333)).then((_) {
          listScrollController.animateTo(
              listScrollController.position.maxScrollExtent,
              duration: const Duration(milliseconds: 333),
              curve: Curves.easeOut);
        });
      } catch (e) {
        // Ignore error, but notify state
        setState(() {});
      }
    }
  }

  String _getCurrentDateTime(DateTime date) {
    // Format date to string
    DateTime thisDate = date;

    String year = thisDate.year.toString();
    String month = thisDate.month.toString().padLeft(2, '0'); // Zero-pad month
    String day = thisDate.day.toString().padLeft(2, '0'); // Zero-pad day
    String hour = thisDate.hour.toString().padLeft(2, '0'); // Zero-pad hour
    String minute =
        thisDate.minute.toString().padLeft(2, '0'); // Zero-pad minute
    String second =
        thisDate.second.toString().padLeft(2, '0'); // Zero-pad second

    return "$year-$month-$day $hour:$minute:$second"; // Return formatted date string
  }
}
