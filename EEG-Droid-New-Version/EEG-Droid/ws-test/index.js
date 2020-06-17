const WebSocket = require('ws')

const connection = new WebSocket('http://192.168.1.125:443')
connection.onmessage = e => {
  console.log(e.data)
}

