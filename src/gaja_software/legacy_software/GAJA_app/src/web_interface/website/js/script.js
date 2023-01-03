// Connecting to ROS
	// -----------------
	var ros = new ROSLIB.Ros();

	// If there is an error on the backend, an 'error' emit will be emitted.
	ros.on('error', function(error) {
		display("Error connecting to websocket server: " + error);
		document.getElementById('connecting').style.display = 'none';
		document.getElementById('connected').style.display = 'none';
		document.getElementById('closed').style.display = 'none';
		document.getElementById('error').style.display = 'inline';
	});

	// Find out exactly when we made a connection.
	ros.on('connection', function() {
		display('Connection made!');
		document.getElementById('connecting').style.display = 'none';
		document.getElementById('error').style.display = 'none';
		document.getElementById('closed').style.display = 'none';
		document.getElementById('connected').style.display = 'inline';
	});

	ros.on('close', function() {
		display('Connection closed.');
		document.getElementById('connecting').style.display = 'none';
		document.getElementById('connected').style.display = 'none';
		document.getElementById('closed').style.display = 'inline';
	});


	let ports = {
	  rosbridge_port: "8061",
	  streamserver_port: "8062"
	};
	let address = window.location.hostname; //prompt("Please enter ip address of Gaja server", "localhost");
	
	
	/*let camera_param = {
	  width: "640",
	  height: "480",
	  pixel_fmt : "mjpeg"	
	};*/
	
	// Create a connection to the rosbridge WebSocket server.
	ros.connect('ws://' + address + ':' + ports.rosbridge_port);

	var cmdSteering = new ROSLIB.Topic({
		ros : ros,
		name : '/gaja_steering',
		messageType : 'gaja_msg/Move_command'
	}); // create gaja_msg with custom msg
	
	var cmdSetCamera = new ROSLIB.Topic({
		ros : ros,
		name : '/cam_config',
		messageType : 'gaja_msg/Cam_config'
	});
   
  
	function sendKeyCommand(dir) {
	
		var move = new ROSLIB.Message({
			pwm : pwmNumber.innerHTML,
			direction : dir
		});
		//alert('UP ' + pwmNumber.innerHTML);
		cmdSteering.publish(move);
		display("Command send: " + "pwm: " + move.pwm + ", direction: " + move.direction);
	};
	
	function sendCamConfig(w, h) {
	
		var setCam = new ROSLIB.Message({
			width : w,
			height : h,
			pixel_fmt : "yuyv"
		});
		
		cmdSetCamera.publish(setCam);
		display("Set camera parameters: " + "w: " + setCam.width + ", h: " + setCam.height);
	};
	
	function camConfig(w, h) {
		sendCamConfig(w, h);
		document.getElementsByName('cameraStream')[0].src = "../iframe_loading.html";
		
		display("Waiting for camera setting...");
		
		var firstPartCamAddr = 'http://' + address + ':' + ports.streamserver_port + '/stream?topic=/usb_cam/image_raw';
		setTimeout(function()
			{ document.getElementsByName('cameraStream')[0].src = firstPartCamAddr + "&width=" + w + "&height=" +  h; display("Done");}, 12000);
	};
	
	btnCustom.onclick = function() {
		const inputCustomWH = document.getElementById('customResTxt').value;
		const customWH = inputCustomWH.split(/[xX]/);
		
		camConfig(customWH[0], customWH[1]);

	};

	btn_320_240.addEventListener("click", function() { camConfig("320", "240"); });
	btn_640_480.addEventListener("click", function() { camConfig("640", "480"); });
	btn_1280_720.addEventListener("click", function() { camConfig("1280", "720"); });
	
	function display(cmd) {
    	document.getElementsByName("log_display")[0].value += (cmd+'\n');
    	document.getElementsByName("log_display")[0].scrollTop = document.getElementsByName("log_display")[0].scrollHeight;
	};

	var slider = document.getElementById("slider");
	var number = document.getElementById("pwmNumber");
	//var btnActive = "#882C2C";
	//const btnClass = document.querySelector('.btn');
	
	slider.oninput = function() {
		number.innerHTML = slider.value;
	};
	
	btnUp.onclick = function() {
  		sendKeyCommand("UP");
	};
	btnDown.onclick = function() {
  		sendKeyCommand("DOWN");
	};
	btnLeft.onclick = function() {
  		sendKeyCommand("LEFT");
	};
	btnRight.onclick = function() {
  		sendKeyCommand("RIGHT");
	};
	btnStop.onclick = function() {
  		sendKeyCommand("STOP");
	};	
	
    
	function btnPress(btnValue) {
		let btn = document.getElementById(btnValue);
		// simulate :active pseudo selector for keydown event
		//let old_bg = btn.style.background;
		//btn.style.background = btnActive;
		// setTimeout(function(){ btn.style.background = old_bg;}, 100);
		btn.classList.add('active');
		setTimeout(function(){ btn.classList.remove('active');}, 100);
		
		btn.click();
	};

	window.addEventListener("keydown", function (event) {
	  if (event.defaultPrevented) {
		return; // Do nothing if the event was already processed
	  }

	  switch (event.key) {
		case "Down": // IE/Edge specific value
		case "ArrowDown":
		case "s":
		  btnPress("btnDown");
		  break;
		case "Up": // IE/Edge specific value
		case "ArrowUp":
		case "w":
		  btnPress("btnUp");
		  break;
		case "Left": // IE/Edge specific value
		case "ArrowLeft":
		case "a":
		  btnPress("btnLeft");
		  break;
		case "Right": // IE/Edge specific value
		case "ArrowRight":
		case "d":
		  btnPress("btnRight");
		  break;
		case " ":
		  btnPress("btnStop");
		  break;
		case "Esc": // IE/Edge specific value
		case "Escape":
		  // Do something for "esc" key press.
		  break;
		default:
		  return; // Quit when this doesn't handle the key event.
	  }

	  // Cancel the default action to avoid it being handled twice
	  event.preventDefault();
	}, true);
	
