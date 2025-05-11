import React, { useState } from "react";
import ProgramItem from "./programitem";

const ProgramContainer = ({ programs, setPrograms }) => {
	const removeProgram = (index) => {
		setPrograms(programs.filter((_, i) => i !== index));
	};

	const sendProgram = (program) => {
		fetch("/api/control", { method: "POST", body: program })
			.then((res) => console.log("Sent:", program))
			.catch((err) => console.error("Error sending program:", err));
	};

	const sendLaunch = () => {
		fetch("/api/control", { method: "POST", body: "launch" })
			.then((res) => console.log("Sent launch command"))
			.catch((err) => console.error("Error sending launch command:", err));
	};

	const sendLand = () => {
		fetch("/api/control", { method: "POST", body: "land" })
			.then((res) => console.log("Sent land command"))
			.catch((err) => console.error("Error sending land command:", err));
	};

	return (
		<div className="program_container modal">
			<h3>Saved Set Points</h3>
			<div className="program_div">
				<ul id="list" className="program_list">
					{programs.map((program, index) => (
						<ProgramItem
							key={program}
							program={program}
							onRemove={() => removeProgram(index)}
							onSend={() => sendProgram(program)}
						/>
					))}
				</ul>
			</div>
			<div className="button_div">
				<button
					type="button"
					className="clear_but"
					onClick={() => setPrograms([])}
				>
					Clear All
				</button>
				<button
					type="button"
					className="launch_but"
					onClick={() => sendLaunch()}
				>
					launch
				</button>
				<button type="button" className="land_but" onClick={() => sendLand()}>
					land
				</button>
			</div>
		</div>
	);
};

export default ProgramContainer;
