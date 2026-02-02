// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {
  private static final StringTopic notificationTopic = NetworkTableInstance.getDefault()
      .getStringTopic("/Elastic/RobotNotifications");
  private static final StringPublisher notificationPublisher = notificationTopic.publish(
      PubSubOption.sendAll(true),
      PubSubOption.keepDuplicates(true));
  private static final StringTopic selectedTabTopic = NetworkTableInstance.getDefault()
      .getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher = selectedTabTopic
      .publish(PubSubOption.keepDuplicates(true));
  private static final ObjectMapper objectMapper = new ObjectMapper();

  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }

  public static void sendNotification(Notification notification) {
    try {
      String json = objectMapper.writeValueAsString(notification);
      notificationPublisher.set(json);
    } catch (JsonProcessingException e) {
      e.printStackTrace();
    }
  }

  public static void sendNotification(String title, String description) {
    sendNotification(new Notification(Notification.NotificationLevel.INFO, title, description));
  }

  public static class Notification {
    @JsonProperty("level")
    private NotificationLevel level;
    
    @JsonProperty("title")
    private String title;
    
    @JsonProperty("description")
    private String description;
    
    @JsonProperty("displayTime")
    private int displayTimeMillis;
    
    @JsonProperty("width")
    private double width = 350;
    
    @JsonProperty("height")
    private double height = -1;

    public Notification(NotificationLevel level, String title, String description) {
      this.level = level;
      this.title = title;
      this.description = description;
      this.displayTimeMillis = 3000;
    }

    public Notification withDisplaySeconds(double seconds) {
      this.displayTimeMillis = (int) (seconds * 1000);
      return this;
    }

    public Notification withWidth(double width) {
      this.width = width;
      return this;
    }

    public Notification withHeight(double height) {
      this.height = height;
      return this;
    }

    public enum NotificationLevel {
      INFO, 
      WARNING,
      ERROR
    }
  }
}