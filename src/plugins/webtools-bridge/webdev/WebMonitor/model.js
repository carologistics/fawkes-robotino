	//-----------------------------------------------Subscription to machine fact
			var topic_name 				= "machine" ;
			var machine_fact_listener 	= new ROSLIB.Topic( {
				ros 			: robot_info_.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + topic_name  ,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: 1000 													  	,
			});

			// eg,fact: (machine (name M-DS) (team MAGENTA) (mtype DS) (incoming) (incoming-agent) (loaded-id 0) (produced-id 0) (x 0.0) (y 0.0) (final-prod-time 0 0) (state IDLE) (sync-id 38))
			machine_fact_listener . subscribe ( 	function(message) {

				for( index_f in message [topic_name_] )
				{
					if( window.team.color == message [topic_name] [index_f] ["team"][0] ) {

						var machine_fact = machines [ message 	 	[topic_name] [index_f]  ];
						var machine_name = machines [ machine_fact 	["name"][0]] ;

						window.machines[machine_name] . prototype . name 			= machine_name ;
						window.machines[machine_name] . prototype . name 			= machine_name ;
						window.machines[machine_name] . prototype . state 			= machine_fact["state"][0] 			;
						window.machines[machine_name] . prototype . type 			= machine_fact["mtype"][0] 			;
						window.machines[machine_name] . prototype . team  			= machine_fact["team"][0]			;
						window.machines[machine_name] . prototype . loaded_id  		= machine_fact["loaded-id"][0]		;
						window.machines[machine_name] . prototype . produced_id 	= machine_fact["produced-id"][0]	;
						window.machines[machine_name] . prototype . final_prod_time = machine_fact["final-prod-time"]	;
						window.machines[machine_name] . prototype . incoming 	 	= machine_fact["incoming"] 			; //array of values of  incoming
						window.machines[machine_name] . prototype . incoming_agent 	= machine_fact["incoming-agent"] 	; //array of values of incoming-agents
					}
				}
			});
