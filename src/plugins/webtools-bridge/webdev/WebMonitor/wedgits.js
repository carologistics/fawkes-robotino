var ros ;

function load(){
	init();
	simple_monitor();
	products();

}

function init() {
  // Connect to ROS.
  ros = new ROSLIB.Ros({
    url : 'ws://localhost:6060'
  });
}


//A simple monitor that allowes live subsscriptions (clips, bb, Ros) Topics. Showes a textaul representation of the topics lively updated. 
function simple_monitor(){

	var div_ids=0;// index to be used to make unique ids

	$("body").append("<div id=monitor_div  class=wedgit container>  </dev>");//the wedgit container
	$("#monitor_div").append("<button class=smplmntr > subscribe </button>");
	$("#monitor_div").append("<input  type='text' class=smplmntr > </input>");

	$(document).ready(function()
	{
	    $("#monitor_div").find(":button") .click(function() 
	    {
	    	var prefiexed_topic_name = $("#monitor_div").find(":text").val(); 

	    	$("#monitor_div").append("<div id=monitor_div_"+div_ids+" class=smplmntr > </div>"); //where the topic wil be displaied

	    	var listener = new ROSLIB.Topic({
			    ros : ros,
			    name : prefiexed_topic_name,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});
		  	listener.div_index=div_ids; //keep the dev's id as a topic attridute
		  	listener.subscribe(function(message) 
		  	{
		  		var dev_id="#monitor_div_"+this.div_index;
		  		
		  		$(dev_id).empty();// clear the div

		  		for (var key in message)//present the content in the div 
		  		{
		  			var dev_content = key + " : ";
		  			if(message[key].constructor === Array)
		  			{
		  				for( var fact in message[key] )
		  				{
		  					dev_content+= JSON.stringify(message[key][fact], null, 5) + "<br>"; //TODO: maybe i dont need to stringfiy 
		  				}
		  			}
		  			else
		  			{
		  				dev_content+= message[key] + "<br>"; 
		  			}
		
		  			dev_content+=  "<br>"; 
		  			$(dev_id).append(dev_content);
		  		}
		  		
		  	});

		  	div_ids++;
	    });

	});
}


function products(){

	var destination_bridge_name= "clips";
	var topic_name= "product";
	var wedgit_id= 	"products_wedgit";
	
	$("body").append("<div id="+wedgit_id+"  class=wedgit container >  </dev>");//the wedgit container
	
	$(document).ready(function()
	{
    	var prefiexed_topic_name = destination_bridge_name +"/" + topic_name ; 

    	var listener = new ROSLIB.Topic({
		    ros : ros,
		    name : prefiexed_topic_name,
		    messageType : 'mm',
		    throttle_rate:1000,
	  	});
	  	//listener.div_index=div_ids; //keep the dev's id as a topic attridute
	  	listener.subscribe(function(message) 
	  	{
	  	//	var dev_id="#mproductsdiv_"+this.div_index;	
	  		$("#"+wedgit_id).empty();// clear the div

	  		for (var key in message)//present the content in the div 
	  		{
	  			if(message[key].constructor === Array)
	  			{

	  				for( var fact in message[key] )
	  				{
	  					var $product_div=$("<div id=product_"+message[key][fact].id+"> </div>");
	  					$product_div.addClass("products");
		  				$("#"+wedgit_id).append($product_div);//dev will hold this product


		  				var base_color = message[key][fact].base[0];
		  				$("#product_"+message[key][fact].id).append("<div class=products_base style= background-color:"+base_color+"> </div>");

		  				for( var ring_index in message[key][fact].rings )
		  				{
		  					var ring_color = message[key][fact].rings[ring_index];
		  					$("#product_"+message[key][fact].id).append("<div class=products_ring style= background-color:"+ring_color+"> </div>");
		  				}
		  				
		  				var cap_color = message[key][fact].cap[0];
		  				$("#product_"+message[key][fact].id).append("<div class=products_cap style= background-color:"+cap_color+"> </div>");
	  				}
	  			}
	  		}
	  	});
	});
}

