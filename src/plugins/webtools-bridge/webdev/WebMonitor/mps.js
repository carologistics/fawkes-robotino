//------------------------------------------------------- MetaData types

function TeamMetadata( options ){

	this . name = options.name 	|| "N/A" ;
	this . color = options.color	|| "N/A" ; 
}


function MpsMetadata( options ){ //Machine data comes as the want

	this . name = options.state				|| "N/A" ;	
	this . state = options.state			|| "N/A" ;	
	this . team = options.team				|| "N/A" ;
	this . img = options.img				|| "N/A" ;
	this . incoming = options.incoming		|| "N/A" ;
	this . incoming_agent = options.incoming_agent || "N/A" ;
	this . produced_id = options.produced_id|| "N/A" ;
	this . loaded_id = options 	.loaded_id	|| "N/A" ;
	
	this . tag_found =false;
	this . explored = false;		//The exploration recognized

	var that = this ;
}

init_mps();

function init_mps(){
	//------------------------------------------------------- Global
	window.team = new TeamMetadata({
		name 	: "Caro", 
		color	: "CYAN"
	});// make  sure you have the team data pretty early

	// this naming is consistent with the naming in the clips facts (machine.. ) 
	if(window.team.color ==  "CYAN" ){

		window.machines  =  { 	"C-BS" 	: new MpsMetadata( {name : "C-BS" } ),
								"C-RS1"	: new MpsMetadata( {name : "C-RS1" } ),
								"C-RS2"	: new MpsMetadata( {name : "C-RS2" } ),
								"C-CS1"	: new MpsMetadata( {name : "C-CS1" } ),
								"C-CS2"	: new MpsMetadata( {name : "C-CS2" } ),
								"C-DS" 	: new MpsMetadata( {name : "C-DS" } )
								
							} ;
	}
	else {
		window.machines  =  { 	"M-BS" 	: new MpsMetadata( {name : "M-BS" } ),
								"M-RS1"	: new MpsMetadata( {name : "M-RS1" } ),
								"M-RS2"	: new MpsMetadata( {name : "M-RS2" } ),
								"M-CS1"	: new MpsMetadata( {name : "M-CS1" } ),
								"M-CS2"	: new MpsMetadata( {name : "M-CS2" } ),
								"M-DS" 	: new MpsMetadata( {name : "M-DS" } )
								
							} ;
		
	}

	
}


//----------------------------------------------------- Monitor

function MpsWedgit ( machine_name ,  $parent )  {

	var	 that 			= this ;
	this. machine_name_	= machine_name;
 	this. wedgit_id_ 	= machine_name + "_wedgit" ; // ex, 'C-RS1_wedgit'
	this. $parent_ 		= $parent ;	
	this. $wedgit_div_	= $("<div>  </div>") 	.addClass("wedgit")	
												.addClass( "mps" )				
												.attr('id', this.wedgit_id_ )
								 				.append( $("<span > </span>" ) . addClass ("header") )
												.append( $("<span > </span>" ) . addClass ("body") )
												.append( $("<span > </span>" ) . addClass ("footer") );
	var $row ;
	var $col ;
	
	this. $wedgit_row_element_ = $("<span> </span>" ) . addClass( "row_element" ) ;	
	this. $wedgit_col_element_ = $("<div> </div>" ) . addClass( "col_element" ) ;

	//construct wedgit header 
	var unprefrixed_machine_name = ( machine_name + "") . split('-') [1] || ( machine_name_ + "") ;
	this.$wedgit_div_ .find(".header") ;


	//=========Machine Title
	$row =  $("<span> </span>" ) .addClass( "row_element" ) ;
	$col =  $("<div> </div>" ) .addClass( "col_element" ) ;
	$col.append ( "<b>" + unprefrixed_machine_name + "</b>" + "<sub>" + "N/A" + "</sub>" ).addClass("machine_name");
	$col.appendTo($row);
	$row. appendTo (this.$wedgit_div_.find(".header"));



	//=================The Extra Machine information
	if(unprefrixed_machine_name == "RS2" || unprefrixed_machine_name == "RS1")
	{
		$row =  $("<span> </span>" ) .addClass( "row_element" ) ;
		$col =  $("<div> </div>" ) .addClass( "col_element" ) ;
		$col.append("<sup> N.A </sup> ").addClass("available_colors");
		$col.appendTo($row);
			
		$col = $("<div> </div>" ) . addClass( "col_element" ) ;
		$col.append("<sup> N.A </sup> ").addClass("selected_color");
		$col.appendTo($row);
		$row. appendTo (this.$wedgit_div_.find(".header"));
	}

	if(unprefrixed_machine_name == "CS2" || unprefrixed_machine_name == "CS1")
	{
		$row =  $("<span> </span>" ) .addClass( "row_element" ) ;
		$col =  $("<div> </div>" ) .addClass( "col_element" ) ;
		$col.append("<sup> N.A </sup> ").addClass("caps_on_shelf");
		$col.appendTo($row);
			
		$col = $("<div> </div>" ) . addClass( "col_element" ) ;
		$col.append("<sup> N.A </sup> ").addClass("cap_loaded");
		$col.appendTo($row);
		$row. appendTo (this.$wedgit_div_.find(".header"));
	}

	if(unprefrixed_machine_name == "BS" )
	{
		$row =  $("<span> </span>" ) .addClass( "row_element" ) ;
		$col =  $("<div> </div>" ) .addClass( "col_element" ) ;
		$col.append("<sup> N.A </sup> ").addClass("active_side");
		$col.appendTo($row);
		$row. appendTo (this.$wedgit_div_.find(".header"));
	}
	

	//Wedgit Body

	//----Exploration stuff
	// //==Row
	$row =  $("<span> </span>" ) .append("<span class = 'row_element' > <b> Exploration: </b> </span>") .addClass( "row_element" ) ;
	$col =  $("<div> </div>" ) . addClass( "col_element" ) ;
	$col.append("<sup> NoTag </sup> ").addClass("tag_found");
	$col.appendTo($row);

	$col = $("<div> </div>" ) . addClass( "col_element" ) ;
	$col.append("<sup> NotExplored </sup> ").addClass("explored");
	$col.appendTo($row);
	$row. appendTo (this.$wedgit_div_.find(".body"));
	//-------------------

	//==Row
	$row =  $("<span> </span>" ) . addClass( "row_element" ) ;
	$col =  $("<div> </div>" ) . addClass( "col_element" ) ;
	$col.append(" <b> loaded_id </b> ").addClass("loaded_id");
	$col.appendTo($row);

	$col = $("<div> </div>" ) . addClass( "col_element" ) ;
	$col.append("<sup> produced_id </sup> ").addClass("produced_id");
	$col.appendTo($row);
	$row. appendTo (this.$wedgit_div_.find(".body"));

	//==title Row
	$row =  $("<span> </span>" ) . addClass( "row_element" ) ;
	$row.append("<b> Incoming: </b> ");
	$row. appendTo (this.$wedgit_div_.find(".body"));

	// //==Row
	 $row =  $("<span> </span>" ) .append("<span class= 'row_element' > Incoming: </span> ");
	 $row. append("<sup> nothing incoming </sup> ") .addClass( "row_element" ) . addClass ("incoming");
	//--agents and resource shown in the same field
	// $col =  $("<div> </div>" ) . addClass( "col_element" ) ;
	// $col.append("<sub> no incoming agents </sub> ").addClass("incoming_agent");
	// $col.appendTo($row);

	// $col = $("<div> </div>" ) . addClass( "col_element" ) ;
	// $col.append("<sub> noting incoming </sub> ").addClass("incoming");
	// $col.appendTo($row);
	$row. appendTo (this.$wedgit_div_.find(".body"));
	


	//EndBody
	//-===================================================
	

	//do I need document .ready here ..Check!
	$(document) . ready (function() {
		that. $parent_ . append ( that .$wedgit_div_ ) ;	
	});


	this . visualize = function () {

		var machine_metadata_ = window . machines [machine_name] ; 
		// take machine name with out team prefix |C-|RS|
		var unprefrixed_machine_name = ( machine_metadata_ . name + "") . split('-') [1] || ( machine_metadata_ . name + "") ;
	
		var $w_header; 
		//==============Title	
		$w_header = $( "<b> " + unprefrixed_machine_name	+ "</b>" + "<sub>" + machine_metadata_ . state 	+ "</sub>" );
		that.$wedgit_div_ . find( "machine_name" )	.empty() .append ( $w_header );

		//==============Machine Info
		if(unprefrixed_machine_name == "RS2" || unprefrixed_machine_name == "RS1")
		{		
			that.$wedgit_div_ . find (".available_colors") .empty();
			if( machine_metadata_ .hasOwnProperty("available_colors") && machine_metadata_ . available_colors.length > 0 )
			{
				for ( i in machine_metadata_.available_colors) 
				{
					that.$wedgit_div_ . find (".available_colors") .append( machine_metadata_ .available_colors [i]  + " | ") ;
				}
			}
			else
			{	
				that.$wedgit_div_ . find (".available_colors" ) .append(" N.A" );
			}

			if(machine_metadata_ .hasOwnProperty("selected_color"))
			{
				that.$wedgit_div_ . find (".selected_color") .empty(). append ( machine_metadata_ .selected_color );
				
				if(machine_metadata_ . selected_color != "NONE")
				{
					that.$wedgit_div_ . find (".selected_color") .addClass("highlight") ;
				}
				else
				{
					that.$wedgit_div_ . find (".selected_color") .removeClass("highlight") ;
				}
			}

			//Dont for get to add the base-loaded
		}


		if(unprefrixed_machine_name == "CS2" || unprefrixed_machine_name == "CS1")
		{		
			that.$wedgit_div_ . find (".caps_on_shelf"). empty();
			
			if(machine_metadata_ .hasOwnProperty ("caps_on_shelf")  && machine_metadata_ .caps_on_shelf > 0 )
			{
				for (var i =0 ; i < machine_metadata_.caps_on_shelf ; i++) 
				{
					that.$wedgit_div_ . find (".caps_on_shelf") .append( machine_metadata_ .assigned_cap_color  + " | ") ;
				 }  
			}
			else
			{	
				that.$wedgit_div_ . find (".caps_on_shelf" ) .append(" None" );
			}

			if(machine_metadata_ .hasOwnProperty ("cap_loaded"))
			{
				that.$wedgit_div_ . find (".cap_loaded"). empty() . append (machine_metadata_ . cap_loaded);
			}

		}	

		if(unprefrixed_machine_name == "BS")
		{		
			if(machine_metadata_ .hasOwnProperty ("active_side") )
			{ 
				that.$wedgit_div_ . find (".active_side"). empty() . append (machine_metadata_ . active_side);
			}
		}	

		var $w_body ;
		that.$wedgit_div_ . find ( $(".loaded_id") ) .empty() .append ( ( ( machine_metadata_.loaded_id )?    machine_metadata_ .loaded_id	: "<sup>Nothing Loaded</sup>") );
		that.$wedgit_div_ . find ( $(".produced_id") ) .empty() .append( ( ( machine_metadata_ .produced_id )?  machine_metadata_ .produced_id : " <sup>Nothing Produced </sup>") ) ;

		that.$wedgit_div_ . find ( $(".incoming") ) .empty();

		if(machine_metadata_ . incoming.length == 0 && machine_metadata_ . incoming_agent.length == 0){
			that.$wedgit_div_ . find ( $(".incoming") ) .append( "<sup> nothing incoming </sup>"  ) ;
		}
		
		var $incoming_content = $("<span></span>");

		if(machine_metadata_ . incoming_agent.length > 0 )
		{
			for ( i in machine_metadata_.incoming_agent) {
				$incoming_content .append( machine_metadata_ .incoming_agent [i]  + " | ") ;
			}
		}

		if(machine_metadata_ . incoming.length > 0 )
		{
			for ( index in machine_metadata_.incoming) {
				$incoming_content .append( machine_metadata_ .incoming [index] + " | " )  ; 
			}
		}


		that.$wedgit_div_ . find ( $(".incoming") ) .append( $incoming_content  ) ;


		//Exploration Stuff
		if (machine_metadata_ . tag_found){
			that.$wedgit_div_ . find ( $(".tag_found") ) .empty() .append( "TagFound" ) ;
		}

		if(machine_metadata_ .explored){
			that.$wedgit_div_ . find ( $(".explored") ) .empty() .append( "Explored" ) ;
		}


	}

}

function MpsMonitor()
{
	
	var that 							= 	this ;
	//to keep track and eliminate redundancy of the state of the data to display in the div when the topic gets published  
	this.	data_to_show_				= 	{} ; 
	// subscribtion_info object parameters
	this.	destination_bridge_name_ 	= 	"clips" ;
	//coming from wedgit object parameters
 	this.	 div_id_ 					= 	"mps_monitor" ;
	this.	$div_ 						= 	$("<div>  </div>")	.	attr('id', this.wedgit_id_ )
																.	addClass("monitor") 
															 	.	addClass( "mps" );

	this.wedgits = {} ;

	
	$("body") . append ( this.$div_ );


	/* To keep the prefixed machine naming convention for consistency with Ros.
		and yet be able to load the mps's widgets on creation on the containing $parent div*/ 
	if( window.team. color == "CYAN" )
	{
		this. wedgits = 
		{
		 	"C-RS1"	: new MpsWedgit( "C-RS1" , this.$div_ ) ,
			"C-RS2"	: new MpsWedgit( "C-RS2" , this.$div_ ) ,
		 	"C-CS1"	: new MpsWedgit( "C-CS1" , this.$div_ ) ,
		 	"C-CS2"	: new MpsWedgit( "C-CS2" , this.$div_ ) ,
		 	"C-DS" 	: new MpsWedgit( "C-DS"  , this.$div_ ) ,
		 	"C-BS" 	: new MpsWedgit( "C-BS"  , this.$div_ ) ,
		} ;		
	}
	else
	{
		this. wedgits = 
		{
		 	"M-RS1"	: new MpsWedgit( "M-RS1" , this.$div_ ) ,
			"M-RS2"	: new MpsWedgit( "M-RS2" , this.$div_ ) ,
		 	"M-CS1"	: new MpsWedgit( "M-CS1" , this.$div_ ) ,
		 	"M-CS2"	: new MpsWedgit( "M-CS2" , this.$div_ ) ,
		 	"M-DS" 	: new MpsWedgit( "M-DS"  , this.$div_ ) ,
		 	"M-BS" 	: new MpsWedgit( "M-BS"  , this.$div_ ) ,
		} ;		
	}


	this.visualize = function( robot_info ) {
			
		var robot_info				=  robot_info;	

		/*Thinking: subscribing to the same topic more than once is totally okay since all the call backs registered will be fired when this topic is 
			published and  added no overload to the server side the only added complexity would be iterating on all of the data just to get what u
			want from them which is less query-able than u want
		*/

		//---------------------------------------------------------------------Subscription to machine fact
		// Fact: (machine (name M-DS) (team MAGENTA) (mtype DS) (incoming) (incoming-agent) (loaded-id 0) (produced-id 0) (x 0.0) (y 0.0) (final-prod-time 0 0) (state IDLE) (sync-id 38))		
		var topic_name = "machine" ;
		var machine_fact_listener 	= new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});

		machine_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "machine"; 

			for( index_f in message [topic_name_] )
			{
				var machine_fact = message  [topic_name_] [index_f]  ;  
				var machine_name = machine_fact ["name"] [0];
		
				if( window.team.color == machine_fact ["team"][0] ) {
					
					window.machines[machine_name] . name 			= machine_fact["name"][0]  			; 	
					window.machines[machine_name] . state 			= machine_fact["state"][0] 			;
					window.machines[machine_name] . type 			= machine_fact["mtype"][0] 			;
					window.machines[machine_name] . team  			= machine_fact["team"][0]			;
					window.machines[machine_name] . loaded_id  		= machine_fact["loaded-id"][0]		;
					window.machines[machine_name] . produced_id 	= machine_fact["produced-id"][0]	;
					window.machines[machine_name] . final_prod_time = machine_fact["final-prod-time"]	;
					window.machines[machine_name] . incoming 	 	= machine_fact["incoming"] 			; //array of values of  incoming
					window.machines[machine_name] . incoming_agent 	= machine_fact["incoming-agent"] 	; //array of values of incoming-agents
				
					that . wedgits [ machine_name ] . visualize();

				}

			}			
				
		});


		//======================================================Zone-Exploitation Fact Subscription
		//(zone-exploration (name Z7) (machine UNKNOWN) (team CYAN) (x 0.0) (y 0.0) (look-pos) (current-look-pos 1) (recognized FALSE) (still-to-explore TRUE) (next nil) (incoming) (incoming-agent) (times-searched 0) (sync-id 20))	
		var topic_name = "zone-exploration" ;
		var zone_exploration_fact_listener 	= new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});

		zone_exploration_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "zone-exploration"; 

			for( index_f in message [topic_name_] )
			{
				var zone_exploration_fact = message  [topic_name_] [index_f]  ;  
				var machine_name = zone_exploration_fact ["machine"] [0];
		
				//To tell the the machine it was found
				if(  machines. hasOwnProperty( machine_name )   && !(machines[machine_name] .explored) && zone_exploration_fact["recognized"] == "TRUE" ) {
					
					window.machines[machine_name] . explored 		= true  ;
					that . wedgits [ machine_name ] . visualize();

				}

			}			
				
		});

		//TODO : unsubscribe when all are found or phase changes



		//======================================================Found Tag Fact Subscription
		//(found-tag (name C-RS1) (side OUTPUT) (frame "/map") (trans -3.122295 4.689034 0.0) (rot 0.0 0.0 0.888741277126459 -0.458409142940703) (sync-id 624140))	
		var topic_name = "found-tag" ;
		var found_tag_fact_listener 	= new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});

		found_tag_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "found-tag"; 

			for( index_f in message [topic_name_] )
			{
				var found_tag_fact = message  [topic_name_] [index_f]  ;  
				var machine_name = found_tag_fact ["name"] [0];
		
				//To tell the the machine it was found
				if(  machines. hasOwnProperty( machine_name )  && !(machines[machine_name] .tag_found) ) {
					
					window.machines[machine_name] . tag_found 	= true  ; 	
					
					that . wedgits [ machine_name ] . visualize();
				}

			}			
				
		});



	//======================================Station information Subscriptions
	//---RING STATION FACT
	//(ring-station (name C-RS2) (available-colors YELLOW ORANGE) (selected-color NONE) (bases-loaded 0) (sync-id 8))	
	topic_name  = "ring-station";
	var ring_station_fact_listener = new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});

		ring_station_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "ring-station";
			
			for( index_f in message [topic_name_] )
			{
				var ring_station_fact = message  [topic_name_] [index_f]  ;  
				var machine_name = ring_station_fact["name"] [0];
		
				// only MpsObject that has the right name accordion to the team colour should be initialized in the machines object. with the right prefix 
				if( machines.hasOwnProperty( machine_name ) ){

					window.machines[machine_name] . available_colors= ring_station_fact["available-colors"]  ; 	
					window.machines[machine_name] . selected_color 	= ring_station_fact["selected-color"][0] ;
					window.machines[machine_name] . bases_loaded 	= ring_station_fact["bases-loaded"][0] 	;
					
					that . wedgits [ machine_name ] . visualize();

				}

			}			
				
		});



	//---CAP STATION FACT
	//(cap-station (name C-CS1) (cap-loaded NONE) (assigned-cap-color GREY) (caps-on-shelf 3) (sync-id 13))	
	topic_name  = "cap-station";
	var cap_station_fact_listener = new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});
		
		cap_station_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "cap-station";
			
			for( index_f in message [topic_name_] )
			{
				cap_station_fact = message  [topic_name_] [index_f]  ;  
				machine_name = cap_station_fact["name"] [0];
		
				// only MpsObject that has the right name accordion to the team colour should be initialized in the machines object. with the right prefix 
				if( machines.hasOwnProperty( machine_name ) ){

					window.machines[machine_name] . cap_loaded 		= cap_station_fact["cap-loaded"][0] ;
					window.machines[machine_name] . assigned_cap_color 	= cap_station_fact["assigned-cap-color"][0] ;
					window.machines[machine_name] . caps_on_shelf 	= cap_station_fact["caps-on-shelf"][0] 	;

					that . wedgits [ machine_name ] . visualize();

				}

			}			
				
		});


	//---BASE STATION FACT
	//(base-station (name C-BS) (active-side INPUT) (sync-id 0))	
	topic_name  = "base-station";
	var base_station_fact_listener = new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});
		
		base_station_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "base-station";
			
			for( index_f in message [topic_name_] )
			{
				base_station_fact = message  [topic_name_] [index_f]  ;  
				machine_name = base_station_fact["name"] [0];
		
				// only MpsObject that has the right name accordion to the team colour should be initialized in the machines object. with the right prefix 
				if( machines.hasOwnProperty( machine_name ) ){

					window.machines[machine_name] . active_side = base_station_fact["active-side"][0] ;
				
					that . wedgits [ machine_name ] . visualize();

				}

			}			
				
		});





	};


 }