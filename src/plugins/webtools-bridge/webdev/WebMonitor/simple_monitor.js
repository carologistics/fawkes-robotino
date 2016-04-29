function simple_monitor(){
	init();

	var div_ids=0;

	$("body").append("<div id=monitor_div  class=wedgit container smplmntr>  </dev>");
	$("#monitor_div").append("<button class=smplmntr > subscribe </button>");
	$("#monitor_div").append("<input  type='text' class=smplmntr > </input>");

	$(document).ready(function()
	{
	    $("#monitor_div").find(":button") .click(function() 
	    {
	    	var topic_name = $("#monitor_div").find(":text").val(); 

	    	$("#monitor_div").append("<div id=monitor_div_"+div_ids+" class=smplmntr > </div>");

	    	var listener = new ROSLIB.Topic({
			    ros : ros,
			    name : topic_name,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});
		  	listener.div_index=div_ids;
		  	listener.subscribe(function(message) {
		  		var parsed_obj=JSON.parse(message.data);

		  		var dev_id="#monitor_div_"+this.div_index;
		  		var dev_content;

		  		for (var key in parsed_obj) 
		  		{
		  			if(parsed_obj[key] === Array)
		  			{
		  				dev_content+= key + ":";
		  				for( var fact in parsed_obj[key])
		  				{
		  					dev_content+= JSON.stringify(fact, null, 5) + "<br>"; 
		  				}

		  			}
		  			else
		  			{
		  				dev_content+= key +":"+ parsed_obj[key] + "<br>"; 
		  			}

		  		}

		  		$(dev_name).text(dev_content);
		  	});

		  	div_ids++;
	    });

	});

}

