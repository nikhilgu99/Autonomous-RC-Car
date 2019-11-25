/*
    Team 12 - Quest 4
    Alex, Justin + Nikhil
    JS is the client and sends messages to the esp32 using UDP Sockets
    Gets messages from the html using socket io
*/
var dgram = require('dgram');

var SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline')

//require('events').EventEmitter.defaultMaxListeners = 100;

var url = require('url');
var http = require('http');
var fs = require('fs');
var io = require('socket.io');  

// Port and IP
var PORT2 = 8081;   //transmit
//var HOST = '192.168.1.145'  //alex's computer => node js
var HOST2 = '192.168.1.142' //justin's computer => esp

// Create socket
var client = dgram.createSocket('udp4');

var server = http.createServer(function(request, response){ //creates the server
	var path = url.parse(request.url).pathname;

    switch(path){
        case '/':
            response.writeHead(200, {'Content-Type': 'text/html'});
            response.write('hello world');
            response.end();
            break;
        case '/remote.html':
            fs.readFile(__dirname + path, function(error, data){
                if (error){
                    response.writeHead(404);
                    response.write("opps this doesn't exist - 404");
                    response.end();
                }
                else{
                    response.writeHead(200, {"Content-Type": "text/html"});
                    response.write(data, "utf8");
                    response.end();
                }
            });
            break;
        default:
            response.writeHead(404);
            response.write("opps this doesn't exist - 404");
            response.end();
            break;
    }
});
server.listen(8080);
var listener = io.listen(server);

listener.sockets.on('connection', function(socket){ //always have our socket open as we are getting the data so we don't have to reopen it to emit to client
    console.log("socket open");

    socket.on('stop', function(msg){ //socket io, receives stop message from web client 

            var stopMsg = "0"; //sends message on port2, acts as the client in this situation
            client.send(stopMsg,0, stopMsg.length,PORT2, HOST2,function(error){
                if(error){
                      console.log('MEH!');
                }
                else{
                  console.log('Sent: ' + stopMsg);
                }
            });

        });
        socket.on('start', function(msg){    //receives the start notification from web client using socket io
            var startMsg = "1";
  //          console.log('time: ' + msg.time);
            client.send(startMsg,0, startMsg.length,PORT2, HOST2,function(error){       //sends to esp
                   if(error){
                        console.log('MEH!');
                    }
                    else{
                        console.log('Sent: ' + startMsg);
                    }
                });
        });

});