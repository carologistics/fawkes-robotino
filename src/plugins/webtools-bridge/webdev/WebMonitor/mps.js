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
	this . incoming_agent = options.incoming_agent|| "N/A" ;
	this . produced_id = options.produced_id|| "N/A" ;
	this . loaded_id = options 	.loaded_id	|| "N/A" ;

	
	var that = this ;
}

	//------------------------------------------------------- Global
	window.team = new TeamMetadata({
		name 	: "Caro", 
		color	: "CYAN"
	});// make  sure you have the team data pretty early

	// this naming is consistent with the naming in the clips facts (machine.. ) 
	window.machines  =  { 	"C-RS1"	: new MpsMetadata( {name : "C-RS1" } ),
							"C-RS2"	: new MpsMetadata( {name : "C-RS2" } ),
							"C-CS1"	: new MpsMetadata( {name : "C-CS1" } ),
							"C-CS2"	: new MpsMetadata( {name : "C-CS2" } ),
							"C-DS" 	: new MpsMetadata( {name : "C-DS" } ),
							"C-BS" 	: new MpsMetadata( {name : "C-BS" } ),

							"M-RS1"	: new MpsMetadata( {name : "M-RS1" } ),
							"M-RS2"	: new MpsMetadata( {name : "M-RS2" } ),
							"M-CS1"	: new MpsMetadata( {name : "M-CS1" } ),
							"M-CS2"	: new MpsMetadata( {name : "M-CS2" } ),
							"M-DS" 	: new MpsMetadata( {name : "M-DS" } ),
							"M-BS" 	: new MpsMetadata( {name : "M-BS" } ),

						} ;





//----------------------------------------------------- Monitor


function MpsWedgit ( machine_name ,  $parent )  {

	var	 that 			= this ;
 	this. wedgit_id_ 	= machine_name + "_wedgit" ; // ex, 'C-RS1_wedgit'
	this. $parent_ 		= $parent ;	
	this. $wedgit_div_	= $("<div>  </div>") 	.addClass("wedgit")	
												.addClass( "mps" )				
												.attr('id', this.wedgit_id_ )
								 				.append( $("<span > </span>" ) . addClass ("header") )
												.append( $("<span > </span>" ) . addClass ("body") )
												.append( $("<span > </span>" ) . addClass ("footer") );
	

	//do I need document .ready here ..Check!
	this. $parent_ . append ( this.$wedgit_div_ ) ;	


	//return the div to be visualized would make sense 
	this . visualize = function () {

		var machine_metadata_ = window . machines [machine_name] ; 

		// $(document) . ready (function() {
		// });

		var unprefrixed_machine_name = ( machine_metadata_ . name + "") . split('-') [1] || ( machine_metadata_ . name + "") ;
		var $header = $( 
			"<b> " + unprefrixed_machine_name	+ "</b>" +	 // take machine name with out team prefix |C-|RS|  
					
			"<sub>" + machine_metadata_ . state 						+ "</sub>" 
		);


		var $content_body = $("<table> </table>") ;
		
		var $tr;
		var $tb;

		$tr = $("<tr></tr>") ;
		$tb = $("<tb></tb>") .append( ( ( machine_metadata_.loaded_id )?    machine_metadata_ .loaded_id	: "no Loaded") );
		$tb.appendTo($tr);
		$tb = $("<tb></tb>") .append( ( ( machine_metadata_ .produced_id )?  machine_metadata_ .produced_id : " no Produced") ) ;
		$tb.appendTo($tr);
		$tr.appendTo($content_body);

		if(machine_metadata_ . incoming.length > 0 )
		{
			$tr = $("<tr></tr>") ;
			for ( index in machine_metadata_.incoming) {
				$tb = $("<tb></tb>") .append( machine_metadata_ .incoming [index]  ) ;
				$tb.appendTo($tr) ; 
			}
			$tr.appendTo($content_body);
		}

		if(machine_metadata_ . incoming_agent > 0 )
		{

			$tr = $("<tr></tr>") ;
			for ( i in machine_metadata_.incoming_agent) {
				$tb = $("<tb></tb>") .append( machine_metadata_ .incoming_agent [i] ) ;
				$tb.appendTo($tr) ; 
			}
			$tr.appendTo($content_body);
		}

		that.$wedgit_div_ . find( ".header" )	. empty();
		that.$wedgit_div_ . find( ".body" )		. empty();
		that.$wedgit_div_ . find( ".footer" )	. empty();

		that.$wedgit_div_ . find( ".header" )	. append ( $header );
		that.$wedgit_div_ . find( ".body" )		. append ( $content_body );
		// that.$wedgit_div_ . find( ".wedgit_footer" ) 	. append ( $header_footer );

		//return $wedgit_div_ ;
	}

}


function MpsMonitor()
{
	
	var that 							= 	this ;

	//to keep track and eliminate redundancy of the state of the data to display in the div when the topic gets published  
	this.	data_to_show_				= 	{} ; 
	
	// subscribtion_info object parameters
	this.	destination_bridge_name_ 	= 	"clips" ;
	this.	topic_name_ 				= 	"" ;
	
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
		
		var topic_name 				= "machine" ;

		var machine_fact_listener 	= new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: 1000 													  	,
		});

		machine_fact_listener . subscribe ( 	function(message) {
			
			for( index_f in message [topic_name] )
			{
				var machine_fact = message  [topic_name] [index_f]  ;  
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



				// if( ! ( that.data_to_show_ . hasOwnProperty ( zone_f ) ) )
				// {
				// 	that.data_to_show_ [zone_f] 	= 	robot_info_.name ; //keep track of the d 
					
				// 	$(document) . ready (function() {
				// 		that.$wedgit_div .find("p") 	.	append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ robot_info_.name +"</sub> &nbsp &nbsp") ;
				// 	});
				// }		


			}			
				
		});

	};


 }