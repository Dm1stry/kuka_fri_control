package application;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

public class KukaOutput implements Output {
	
	private ITaskLogger logger;
	
	public KukaOutput(ITaskLogger logger)
	{
		this.logger = logger;
	}
	
	@Override
	public void print(String message)
	{
		logger.info(message);
	}
	
	@Override
    public void printerr(String message)
	{
		logger.error(message);
	}
}
