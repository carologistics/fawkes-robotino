

//A simple monitor that allowes live subsscriptions (clips, bb, Ros) Topics. Showes a textaul representation of the topics lively updated. 
function simple_monitor(){

	var div_ids=0;// index to be used to make unique ids

	window.$layout_container.append("<div id=monitor_div  class=wedgit container>  </dev>");//the wedgit container
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
	
// 	window.$layout_container.append("<div id="+wedgit_id+"  class=wedgit>  </div>");//the wedgit container
	
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


	window.$layout_container.append($wedgit_div);//the wedgit container
	

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
	  		$("#"+wedgit_id).append("<h2> Products:</h2>");

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



						var $product_div=$("#product_"+product_facts.product[product]["product-id"]).clone();//Make a similare DIV
						$product_div.attr('id',"product_"+product_facts.product[product].id);
						
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


	window.$layout_container.append($wedgit_div);//the wedgit container
	

	$(document).ready(function()
	{

	//to listen to the  incase the product widget not instialized
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
	  		$("#"+wedgit_id).append("<h2> Orders: </h2>");

	  		order_facts= message;

	  		if( product_facts != null)
	  		{
				for( var order in order_facts.order )
				{
					for( var product in product_facts.product )
					{
						if (JSON.parse(product_facts.product[product].id) == JSON.parse( order_facts.order[order]["product-id"] ))
		  				{
							var $order_div=$("<div> </div>").addClass("order");// the class order is given to  each one of the orders_divs that contain the product div 
							var $product_div=$("<div> </div>").addClass("product").attr('id',"product_"+product_facts.product[product].id);// the class 'produced' is given to the div containing  peace itself
							
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



function robotInfo( robot_name , bridge_connection )
{
	//var robot_name="Robot 1";
	var destination_bridge_name= "clips";
	var provided_tools= { tasks:true , state:true , lock_role: true , locked_resource: true , skills : true , holding : true} ;
	var container_id=	"robot_info__"+robot_name;

	var $container_div= $("<div>  </div>").addClass("container").attr('id',container_id);// div contaning the wedgi
	window.$layout_container.append($container_div);//the wedgit container
	
	var $container_header= $("<div>  </div>").addClass("container_header");
	$container_header.append($("<h1>"+robot_name+"</h1>").addClass("container_header_element"));
	$container_div.append($container_header);

	
	$(document).ready(function()
	{
		
		//------------The State Wedgit
		if(provided_tools["lock_role"])
		{
		  	var wedgit_id = "lock_role"+"__"+robot_name;
			var $lock_role_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).addClass("wedgit").addClass("container_header_element");// div contaning the wedgit

			var old_lock_role = "something";

			var lock_role_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "lock-role" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	lock_role_fact_listener.subscribe(function(message){

		  		if(old_lock_role != message["lock-role"][0].fields[0])
		  		{
		  			$lock_role_wedgit_div.empty();
		  			$lock_role_wedgit_div.html(' <img> </img>');
		  			//$lock_role_wedgit_div.html("<span>" +  message["lock-role"][0].fields[0] + "</span>");
		  			if ( message["lock-role"][0].fields[0] == "MASTER")
		  			{
		  				$lock_role_wedgit_div.addClass("master");
		  			}
		  			else
		  			{
		  				$lock_role_wedgit_div.addClass("slave");
		  				$lock_role_wedgit_div.html("<span>" +  message["lock-role"][0].fields[0] + "</span>");

		  			}


		  			old_lock_role = message["lock-role"][0].fields[0];
		  		}

		  	});

		  	$container_header.append($lock_role_wedgit_div);
			
		}


		//=----------Holding Wedgit
		if(provided_tools["holding"])
		{
		  	var wedgit_id = "holding"+"__"+robot_name;
			var $holding_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).addClass("wedgit").addClass("container_header_element");// div contaning the wedgit

			var old_holding = "NONE";

			var holding_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "holding" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	holding_fact_listener.subscribe(function(message){

		  		if(old_holding != message.holding[0].fields[0])
		  		{
		  			old_holding= message.holding[0].fields[0];
		  		}

		  	});


		  	var product_facts_listener = new ROSLIB.Topic({
		    ros : ros,
		    name : destination_bridge_name +"/" + "product" ,
		    messageType : 'mm',
		    throttle_rate:1000,
	  		});


	  		product_facts_listener.subscribe(function(message) 
	  		{
		  		$holding_wedgit_div.empty();// clear the div

		  		if( old_holding != "NONE")
		  		{
		  			for( var product in message.product )
					{

						if (JSON.parse(message.product[product]["id"]) == old_holding )//A product that describes a production process
						{

							var $product_div=$("<div> </div>");//Make a similare DIV

							var base_color = message.product[product].base[0];
							$product_div.prepend("<div class=products_base style= background-color:"+base_color+"> </div>");
							
							for( var ring_index in message.product[product].rings )
							{
								var ring_color = message.product[product].rings[ring_index];
								$product_div.prepend("<div class='products_ring "+ring_index +" ' style= background-color:"+ring_color+"> </div>");
							}
							
							var cap_color = message.product[product].cap[0];
							$product_div.prepend("<div class=products_cap style= background-color:"+cap_color+"> </div>");

							$holding_wedgit_div.append($product_div); //div will hold this product	  				

						}
						
					}
		  		}
		  	});



		  	$container_header.append($holding_wedgit_div);
			
		}




		//------------The Role Wedgit

		if(provided_tools["state"])
		{
		  	var wedgit_id = "state"+"__"+robot_name;
			var $state_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).addClass("wedgit").addClass("container_header_element");// div contaning the wedgit

			var old_state = "something";

			var state_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "state" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	state_fact_listener.subscribe(function(message){

		  		if(old_state != message.state[0].fields[0])
		  		{
		  			$state_wedgit_div.empty();
		  			$state_wedgit_div.html("<sub>" + message.state[0].fields[0] + "</sub>");
		  			old_state = message.state[0].fields[0];
		  		}

		  	});

		  	$container_header.append($state_wedgit_div);
			
		}


		//------------The Task Wedgit

		if(provided_tools["tasks"])
		{
			var wedgit_id = "tasks"+"__"+robot_name;
			var $task_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).append("<h2> Running Task: </h2>").append("<p> </p>").append("<ol> </ol>").addClass("wedgit");// div contaning the wedgit

			var running_task;

			var task_facts_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "task" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	task_facts_listener.subscribe(function(message){

		  		running_task=null;

		  		for ( task_index in message.task)
		  		{
		  			var task = message.task[task_index];
		  			if(	task.state == "running")
		  			{
		  				running_task=task;
		  				$task_wedgit_div.find("p").empty().html("<b> "+task.name+"</b>"+"  "+"<sup> priority:"+JSON.parse(task.priority)+"</sup>");
		  				break;
		  			}

		  		}

	  			if(!running_task)
	  			{
	  				$task_wedgit_div.find("p").empty();
	  				$task_wedgit_div.find("ol").empty();// to delete the details if there was an old task
	  				$task_wedgit_div.find("p").text(" No running Task ");
	  			}


		  	});

			var step_facts_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "step" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	step_facts_listener.subscribe(function(message){

	  			if(running_task)
	  			{
	  				var $ol_element  = $("<ol></ol>");

	  				for (task_step_index in running_task.steps )
	  				{

			  			for( step_index in message.step )
			  			{
			  				var step = message.step[step_index];
			  				// var step_order= $.inArray( step.id[0] , running_task.steps  );
			  				if ( step.id[0] == running_task.steps[task_step_index] )
			  				{
			  					//ex (step (id 911606601) (name get-output) (state failed) (task-priority 50) (machine C-CS1) (zone nil) (product-type nil) (machine-feature CONVEYOR) (shelf-slot LEFT) (base BLACK) (ring BLUE) (cs-operation MOUNT_CAP) (gate 1) (product-id 0))
			  					var $li_content=$("<span>" +"<b>"+step.name+"</b>"  +"<sup>"+step["task-priority"]+"</sup>"  +" machine:"+"<b>"+step.machine+"</b>"   +" feature:"+"<b>"+step["machine-feature"]+"</b>"  +" shelf:"+"<b>"+step["shelf-slot"]+"</b>"   +" base:"+"<b>"+step["base"]+"</b>"  +"</span>");
			  					if(step.state == "running") $li_content.addClass("highlight");

			  					var $li_element = $("<li> </li>" ).append($li_content);

			  					$ol_element.append($li_element );
			  				}
			  			}

	  				}


		  			$task_wedgit_div.find("ol").remove();// to prepare for the refresh
		  			$task_wedgit_div.append($ol_element);// to prepare for the refresh
		  		}

		  	});

		  	$container_div.append($task_wedgit_div);
		}


		//------------Locked resources wedgit
		if(provided_tools["locked_resource"])
		{
			var wedgit_id = "locked_resources"+"__"+robot_name;
			var $locked_resource_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).append("<h2> Locked Resources: </h2>").append("<ul> </ul>").addClass("wedgit");// div contaning the wedgit


			var locked_resources_facts_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "locked-resource" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	locked_resources_facts_listener.subscribe(function(message){

			  	$locked_resource_wedgit_div.find("ul").empty();// refresh and delete unlocked resources"<span>"+fact.resource+"</span>"
			  	var something_locked;

		  		for ( fact_index in message["locked-resource"])
		  		{
		  			var fact = message["locked-resource"][fact_index];
		  			if(	fact.agent == robot_name)
		  			{
		  				something_locked = true;
		  				var $li_element  = $("<li></li>");
		  				$li_element.html("<span>"+fact.resource+"</span>");
	
		  				$locked_resource_wedgit_div.find("ul").append($li_element);
		  			}
		  		}

		  		if (!something_locked)
		  		{
		  			$locked_resource_wedgit_div.find("ul").append("<li> <span> No Locked Resources</span> </li>")

		  		}

		  	});

		  	$container_div.append($locked_resource_wedgit_div);
		}

		//--------------skills wedgit
		if(provided_tools["skills"])
		{
			var wedgit_id = "skills"+"__"+robot_name;
			var $skills_wedgit_div= $("<div>  </div>").attr('id',wedgit_id).addClass("wedgit") // div contaning the wedgit
				.append("<h2> Skills: </h2>")
				.append("<h3 > to-excute:&nbsp </h3>") .append("<p class=execute> </p> <br>")
				.append("<h3 > Running:&nbsp </h3>") .append("<p class=running> </p> <br>")
				.append("<h3 > Last Done:&nbsp </h3>") .append("<p class=done> </p> <br>")


			var skill_to_excute_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "skill-to-execute" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	skill_to_excute_fact_listener.subscribe(function(message){

			  	$skills_wedgit_div.find("p.execute").empty();

		  		for ( fact_index in message["skill-to-execute"])
		  		{
		  			var fact = message["skill-to-execute"][fact_index];

		  			$skills_wedgit_div.find("p.execute").append("<pa>"+ fact["skill"] +"&nbsp &nbsp</b>");
		  			for (args_index in fact["args"])
		  			{
		  				if(args_index% 2)
		  				{
		  					$skills_wedgit_div.find("p.execute span").append( "<strong>"+fact["args"][args_index]+"</strong> &nbsp " );
		  				}
		  				else
		  				{
		  					//$skills_wedgit_div.find("p.execute span").append( fact["args"][args_index] );
		  				}
		  			}
		  		
		  		}

		  	});


			var skill_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "skill" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	skill_fact_listener.subscribe(function(message){

			  	$skills_wedgit_div.find("p.running").empty();// refresh and delete unlocked resources"<span>"+fact.resource+"</span>"

		  		for ( fact_index in message["skill"])
		  		{

		  			var fact = message["skill"][fact_index];
		  			if(	fact.status == "RUNNING" )
		  			{

		  				$skills_wedgit_div.find("p.running").append($("<span>"+ message["skill"][fact_index]["skill-string"] +"</span>").addClass("highlight"));
		  			}
					else
					{
			  			$skills_wedgit_div.find("p.running").append("<span> "+ message["skill"][fact_index]["skill-string"] +"<sub>"+message["skill"][fact_index]["state"] +"</sub>"+"</span>")
					}

		  		}

		  	});


		  	var skill_done_fact_listener = new ROSLIB.Topic({
			    ros : bridge_connection ,
			    name : destination_bridge_name +"/" + "skill-done" ,
			    messageType : 'mm',
			    throttle_rate:1000,
		  	});

		  	skill_done_fact_listener.subscribe(function(message){

			  	$skills_wedgit_div.find("p.done").empty();

		  		for ( fact_index in message["skill-done"])
		  		{
		  			var fact = message["skill-done"][fact_index];
		  			var $span = $("<span>"+ fact["name"] +"</span> <br>");
		  			$span.addClass(fact["status"][0]);
		  			$skills_wedgit_div.find("p.done").append($span);
		  		}

		  	});



		  	$container_div.append($skills_wedgit_div);

		}





	});


}