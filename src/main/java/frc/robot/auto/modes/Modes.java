package frc.robot.auto.modes;
import frc.robot.auto.tasks.Tasks;

import java.util.ArrayList;

public abstract class Modes {
  private ArrayList<Tasks> m_tasks;

  public Modes() {
    m_tasks = new ArrayList<>();
  }

  public Tasks getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Tasks task) {
    m_tasks.add(task);
  }

  public void queueTasks(ArrayList<Tasks> tasks) {
    for (Tasks task : tasks) {
      m_tasks.add(task);
    }
  }

  public void queueEnd() {
    //queueTask(new DriveForwardTask(0, 0));
  }

  public abstract void queueTasks();
}