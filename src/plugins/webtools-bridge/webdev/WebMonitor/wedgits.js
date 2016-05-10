var ros ;
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts

function load(){
	init();
	simple_monitor();
	orders();
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


// function products(){

// 	var destination_bridge_name= "clips";
// 	var topic_name= "product";
// 	var wedgit_id= 	"products_wedgit";
	
// 	$("body").append("<div id="+wedgit_id+"  class=wedgit>  </div>");//the wedgit container
	
// 	$(document).ready(function()
// 	{
//     	var prefiexed_topic_name = destination_bridge_name +"/" + topic_name ; 

//     	var listener = new ROSLIB.Topic({
// 		    ros : ros,
// 		    name : prefiexed_topic_name,
// 		    messageType : 'mm',
// 		    throttle_rate:1000,
// 	  	});
// 	  	listener.subscribe(function(message) 
// 	  	{
// 	  		$("#"+wedgit_id).empty();// clear the div

// 	  		for (var key in message)//present the content in the div 
// 	  		{
// 	  			if(message[key].constructor === Array)
// 	  			{

// 	  				for( var fact in message[key] )
// 	  				{
// 	  					var $product_div=$("<div> </div>");
// 	  					$product_div.addClass("products").attr('id',"product_"+message[key][fact].id);
		  				
// 		  				var base_color = message[key][fact].base[0];
// 		  				$product_div.prepend("<div class=products_base style= background-color:"+base_color+"> </div>");
		  				
// 		  				for( var ring_index in message[key][fact].rings )
// 		  				{
// 		  					var ring_color = message[key][fact].rings[ring_index];
// 		  					$product_div.prepend("<div class=products_ring style= background-color:"+ring_color+"> </div>");
// 		  				}
		  				
// 		  				var cap_color = message[key][fact].cap[0];
// 		  				$product_div.prepend("<div class=products_cap style= background-color:"+cap_color+"> </div>");


		  				
// 		  				$("#"+wedgit_id).append($product_div); //div will hold this product
// 	  				}
// 	  			}
// 	  		}
// 	  	});
// 	});
// }



function products()
{
	var destination_bridge_name= "clips";
	var product_topic_name= "product";
	var wedgit_id= 	"products_wedgit";

	var $wedgit_div= $("<div>  </div>").addClass("widget").attr('id',wedgit_id);// div contaning the wedgit


	$("body").append($wedgit_div);//the wedgit container
	

	$(document).ready(function()
	{

    	var product_facts_listener = new ROSLIB.Topic({
		    ros : ros,
		    name : destination_bridge_name +"/" + product_topic_name ,
		    messageType : 'mm',
		    throttle_rate:1000,
	  	});

	  	product_facts_listener.subscribe(function(message) 
	  	{
	  		product_facts=message;
	  		$("#"+wedgit_id).empty();// clear the div
	  		$("#"+wedgit_id).append("<h1> Products:</h1>");

	  		if( order_facts != null)
	  		{
	  			for( var product in product_facts.product )
				{

					if (JSON.parse(product_facts.product[product]["product-id"]) !=0 )//A product that describes a production process
					{
						
						var related_order; 
						for( var product_index2 in product_facts.product)
						{
							if (JSON.parse(product_facts.product[product_index2].id) == JSON.parse(product_facts.product[product]["product-id"]) )
			  				{
			  					related_order = product_facts.product[product_index2];				
			  				}
						}


						$("#order_"+product_facts.product[product]["product-id"])// Mark this order as active

						var $product_div=$("#product_"+product_facts.product[product]["product-id"]).clone();//Make a similare DIV
						$product_div.attr('id',"product_"+product_facts.product[product].id).addClass("product").removeClass("order");
						
						$product_div.children().addClass("part_processing");//set all the part to still processing

						if (product_facts.product[product].base[0] == related_order.base[0] )
						{
							$product_div.children(".products_base").removeClass("part_processing").addClass("part_complete");
						}
						 
						
						for( var ring_index in product_facts.product[product].rings )
						{
							if (product_facts.product[product].rings[ring_index] == related_order.rings[ring_index])
							{
						 		$product_div.children(".products_ring."+ring_index).removeClass("part_processing").addClass("part_complete");
							}
						}
						
						if (product_facts.product[product].cap[0] == related_order.cap[0])
						{
							$product_div.children(".products_cap").removeClass("part_processing").addClass("part_complete");
						}

						$("#"+wedgit_id).append($product_div); //div will hold this product	  				

					}
					
				}
	  		}
	  	});
	});

}


function orders(){

	var destination_bridge_name= "clips";
	var order_topic_name= "order";
	var product_topic_name= "product";
	var wedgit_id= 	"orders_wedgit";

	var $wedgit_div= $("<div>  </div>").addClass("widget").attr('id',wedgit_id);// div contaning the wedgit


	$("body").append($wedgit_div);//the wedgit container
	

	$(document).ready(function()
	{

    // 	var product_facts_listener = new ROSLIB.Topic({
		  //   ros : ros,
		  //   name : destination_bridge_name +"/" + product_topic_name ,
		  //   messageType : 'mm',
		  //   throttle_rate:1000,
	  	// });

	  	// product_facts_listener.subscribe(function(message) 
	  	// {
	  	// 	product_facts=message;

	  	// });


    	var order_facts_listener = new ROSLIB.Topic({
		    ros : ros,
		    name : destination_bridge_name +"/" + order_topic_name ,
		    messageType : 'mm',
		    throttle_rate:1000,
	  	});
	  	order_facts_listener.subscribe(function(message) 
	  	{
	  		$("#"+wedgit_id).empty();// clear the div
	  		$("#"+wedgit_id).append("<h1> Orders: </h1>");

	  		order_facts= message;

	  		if( product_facts != null)
	  		{
				for( var order in order_facts.order )
				{
					for( var product in product_facts.product )
					{
						if (JSON.parse(product_facts.product[product].id) == JSON.parse( order_facts.order[order]["product-id"] ))
		  				{
							var $order_div=$("<div> </div>").addClass("order");
							var $product_div=$("<div> </div>").addClass("product").attr('id',"product_"+product_facts.product[product].id);
							
							var base_color = product_facts.product[product].base[0];
							$product_div.prepend("<div class=products_base style= background-color:"+base_color+"> </div>");
							
							for( var ring_index in product_facts.product[product].rings )
							{
								var ring_color = product_facts.product[product].rings[ring_index];
								$product_div.prepend("<div class='products_ring "+ring_index +" ' style= background-color:"+ring_color+"> </div>");
							}
							
							var cap_color = product_facts.product[product].cap[0];
							$product_div.prepend("<div class=products_cap style= background-color:"+cap_color+"> </div>");


							//order info
							if(JSON.parse( order_facts.order[order]["in-production"][0] ) == 1)
							{
								$order_div.addClass("active-order")
							}

							var $order_info=$("<div> </div>").addClass("order_info");
							$order_info.append("<span> Gate: </span>").append("<span>"+ order_facts.order[order]["delivery-gate"][0]+"</span>").append("<br>");
							$order_info.append("<span>"+ order_facts.order[order]["begin"][0] + ":" + order_facts.order[order]["end"][0]+"</span>").append("<br>");
							$order_info.append("<span>"+ order_facts.order[order]["quantity-delivered"][0] + "/" + order_facts.order[order]["quantity-requested"][0]+"</span>").append("<br>");

							$order_div.append($product_div);
							$order_div.append($order_info);

							$("#"+wedgit_id).append($order_div); //div will hold this product	  				

		  				}
					}
				}
	  		}
	  	});
	});
}


