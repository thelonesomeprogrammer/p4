import React from "react";

const ProgramItem = ({ program, onRemove, onSend }) => {
	return (
		<li>
			<p>{program}</p>
			<button type="button" className="send_button" onClick={onSend}>
				send
			</button>
			<button type="button" className="rm_button" onClick={onRemove}>
				X
			</button>
		</li>
	);
};

export default ProgramItem;
