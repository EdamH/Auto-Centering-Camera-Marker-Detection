import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';
import './helpers/Message.dart';

// Main widget for the Graph Page
class GraphPage extends StatefulWidget {
  final List<Message> messages; // List of messages to be visualized

  const GraphPage({required this.messages}); // Constructor requiring messages

  @override
  _GraphPageState createState() =>
      _GraphPageState(); // Create state for the widget
}

class _GraphPageState extends State<GraphPage> {
  @override
  Widget build(BuildContext context) {
    // Get the screen width to set chart width
    final screenWidth = MediaQuery.of(context).size.width;

    return Scaffold(
      appBar: AppBar(
        title: const Text(
          'Daily Devs Histogram',
          style: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.bold,
            fontSize: 20.0,
          ),
        ),
        backgroundColor: Colors.deepPurpleAccent, // Set app bar color
        centerTitle: true, // Centers the title
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0), // Padding around the body
        child: Column(
          children: [
            Expanded(
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal, // Allow horizontal scrolling
                child: Container(
                  width: screenWidth, // Set container width to screen width
                  child: BarChart(
                    BarChartData(
                      gridData: FlGridData(show: false), // Disable grid lines
                      titlesData: FlTitlesData(
                        bottomTitles: AxisTitles(
                          sideTitles: SideTitles(
                            showTitles: true,
                            reservedSize: 60, // Reserve space for titles
                            getTitlesWidget: (value, meta) {
                              List<String> titles =
                                  _getBarTitles(); // Get bar titles
                              return SideTitleWidget(
                                axisSide: meta.axisSide,
                                child: RotatedBox(
                                  quarterTurns: 1, // Rotate titles 90 degrees
                                  child: Text(
                                    titles[value
                                        .toInt()], // Get the title for this bar
                                    style: TextStyle(
                                        fontSize: 12), // Set title style
                                  ),
                                ),
                              );
                            },
                          ),
                        ),
                      ),
                      borderData:
                          FlBorderData(show: true), // Show chart borders
                      barGroups: _getChartData(), // Get chart data
                      maxY: _getMaxY(), // Get maximum Y value for the chart
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // Aggregate message counts by day
  Map<String, int> _aggregateByDay() {
    Map<String, int> dayCounts = {}; // Map to hold counts by date

    // Iterate through messages
    for (var message in widget.messages) {
      DateTime timestamp = message.timestamp; // Get message timestamp
      String date = _getDateString(timestamp); // Format date as a string

      // Update the count for this date
      if (dayCounts.containsKey(date)) {
        dayCounts[date] = dayCounts[date]! + 1; // Increment count
      } else {
        dayCounts[date] = 1; // Initialize count
      }
    }

    return dayCounts; // Return the aggregated counts
  }

  // Convert DateTime to a formatted string (YYYY-MM-DD)
  String _getDateString(DateTime date) {
    String year = date.year.toString();
    String month = date.month.toString().padLeft(2, '0'); // Zero-pad month
    String day = date.day.toString().padLeft(2, '0'); // Zero-pad day
    return "$year-$month-$day"; // Return formatted date
  }

  // Generate data for the bar chart
  List<BarChartGroupData> _getChartData() {
    Map<String, int> dayCounts = _aggregateByDay(); // Get aggregated day counts
    List<BarChartGroupData> bars = []; // List to hold bar chart data

    int index = 0;
    dayCounts.forEach((date, count) {
      bars.add(BarChartGroupData(
        x: index, // Set bar index
        barRods: [
          BarChartRodData(
            toY: count.toDouble(), // Set height of the bar
            color: Colors.blue, // Set bar color
            width: 15, // Set bar width
          ),
        ],
      ));
      index++; // Increment index for next bar
    });

    return bars; // Return the list of bar chart data
  }

  // Get titles for the bars
  List<String> _getBarTitles() {
    Map<String, int> dayCounts = _aggregateByDay(); // Get day counts
    List<String> titles = []; // List to hold titles

    dayCounts.keys.forEach((date) {
      titles.add(date); // Add each date to titles
    });

    return titles; // Return the list of titles
  }

  // Get the maximum Y value for the chart
  double _getMaxY() {
    Map<String, int> dayCounts = _aggregateByDay(); // Get day counts
    int maxCount =
        dayCounts.values.reduce((a, b) => a > b ? a : b); // Find max count
    return maxCount.toDouble(); // Return as double
  }
}
