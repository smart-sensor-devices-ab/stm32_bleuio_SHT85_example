import * as my_dongle from 'bleuio'

//connect to BleuIO
document.getElementById('connect').addEventListener('click', function(){
  my_dongle.at_connect()
})
//get sensor data
document.getElementById('getdata').addEventListener('click', function(){
  document.getElementById('loader').innerHTML = 'Loading'
  //set the BleuIO dongle into dual role
    my_dongle.at_dual().then(()=>{
      // sensor id of the device that we are trying to get data from
      let sensorID='05084FA3'

      //look for advertised data of with the sensor id
        my_dongle.at_findscandata(sensorID,4).then(x=>{        

          //split the advertised data from the respnse
          let advdata= x[x.length-1].split(" ").pop()

          //trim the advertised string to only get sensor response
          const result = advdata.split(sensorID).slice(1).join(sensorID) 

          //get temperature and humidity value
          let temp = result.substring(0, 4);
          let hum = result.substring(4, 8);

          //convert from hex to decimal and device by 100
          temp = parseInt(temp, 16)/100
          hum = (parseInt(hum, 16)/100).toFixed(1)  

          document.getElementById('loader').innerHTML = ''
          document.getElementById('response').innerHTML = `Sensor ID : 05084FA3 <br/>
          Temperature : ${temp} °C<br/>
          Humidity : ${hum} %rH<br/>`              
        })
    })
    
  })
