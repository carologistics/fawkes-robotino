function simple_monitor(){
	init();

	var div_ids=0;
	$("body").append("<button id=monitor_btn  class=smplmntr > subscribe </button>");
	$("body").append("<input id=monitor_inp type='text' class=smplmntr > </input>");

	$(document).ready(function(){
	    $("#monitor_btn").click(function() {
	    	var topic_name = $("#monitor_inp").val();

	    	$("body").append("<div id=d_"+div_ids+" class=smplmntr > </div>");

	    	
	    	var listener = new ROSLIB.Topic({
			    ros : ros,
			    name : topic_name,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});
		  	listener.div_id=div_ids;
		  	listener.subscribe(function(message) {
		  		var dev_name="#d_"+this.div_id;
		  		$(dev_name).text(JSON.stringify(message, null, 5));
		  	});

		  	div_ids++;
	    });

	});

}

