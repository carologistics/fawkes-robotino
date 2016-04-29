function simple_monitor(){
	init();

	var div_ids=0;

	$("body").append("<div id=monitor_div  class=wedgit container>  </dev>");
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
		  		var parsed_obj=message;

		  		var dev_id="#monitor_div_"+this.div_index;
		  		$(dev_id).empty();

		  		for (var key in parsed_obj) 
		  		{
		  			var dev_content = key + " : ";
		  			if(parsed_obj[key].constructor === Array)
		  			{
		  				
		  				for( var fact in parsed_obj[key])
		  				{
		  					dev_content+= JSON.stringify(parsed_obj[key][fact], null, 5) + "<br>"; 
		  				}

		  			}
		  			else
		  			{
		  				dev_content+= parsed_obj[key] + "<br>"; 
		  			}
		  			
		  			dev_content+=  "<br>"; 
		  			
		  			$(dev_id).append(dev_content);
		  		}

		  		
		  	});

		  	div_ids++;
	    });

	});

}

