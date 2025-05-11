import React, { useState, useEffect } from "react";

const DataInput = ({ onAddProgram, cords, link }) => {
	const [setpoint, setSetpoint] = useState("");

	const handleAddSetpoint = () => {
		if (setpoint.trim()) {
			onAddProgram(setpoint.trim());
			setSetpoint(""); // Clear the input field
		} else {
			alert("Setpoint cannot be empty");
		}
	};

	const handleFileChange = (event) => {
		const file = event.target.files[0];
		const reader = new FileReader();
		reader.onload = function () {
			const lines = this.result.split("\n");
			for (line of lines) {
				if (line.trim()) {
					onAddProgram(line.trim());
				}
			}
		};
		reader.readAsText(file);
	};

	return (
		<div className="modal">
			<h3>General Data And Control</h3>
			<div className="data_div">
				<p>cf link:</p>
				<p id="link">{link}</p>
				<p>cf position:</p>
				<p id="cords">{cords}</p>
				<p>add setpoint:</p>
				<div className="input_div">
					<input
						name="req"
						id="req"
						type="text"
						value={setpoint}
						onChange={(e) => setSetpoint(e.target.value)}
					/>
					<button type="button" onClick={handleAddSetpoint}>
						+
					</button>
				</div>
				<label htmlFor="file">setpoint save file:</label>
				<input type="file" name="file" id="file" onChange={handleFileChange} />
			</div>
		</div>
	);
};

export default DataInput;
