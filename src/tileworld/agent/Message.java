package tileworld.agent;

public class Message {
	private String from; // the sender
	private String to; // the recepient
	private String message; // the message

	private TWAgent agent;
	
	public Message(String from, String to, String message, TWAgent agent){
		this.from = from;
		this.to = to;
		this.message = message;
		this.agent = agent;
	}

	public String getFrom() {
		return from;
	}

	public String getTo() {
		return to;
	}

	public String getMessage() {
		return message;
	}

	public TWAgent getAgent() { return agent; }
}
