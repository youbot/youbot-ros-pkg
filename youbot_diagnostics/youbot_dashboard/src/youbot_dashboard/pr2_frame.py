# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# modified by: Frederik Hegger
# date: 29.11.2012
#

import roslib
roslib.load_manifest('youbot_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
from pr2_msgs.msg import PowerState, PowerBoardState, DashboardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import std_msgs.msg
import std_srvs.srv

import rospy
from roslib import rosenv

from os import path
import threading

from status_control import StatusControl
from youbot_status_control import youbotStatusControl
from power_state_control import PowerStateControl
from diagnostics_frame import DiagnosticsFrame
from rosout_frame import RosoutFrame

class PR2Frame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='youbot_dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('youbot_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("youbot_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
        
        self.SetTitle('youbot_dashboard (%s)'%rosenv.get_master_uri())
        
        icons_path = path.join(roslib.packages.get_pkg_dir('pr2_dashboard'), "icons/")
        youbot_icons_path = path.join(roslib.packages.get_pkg_dir('youbot_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagn."), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, icons_path, "diag", True)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)
        
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Platform"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # base status
        self._base_status = youbotStatusControl(self, wx.ID_ANY, youbot_icons_path, "base", True)
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: Stale"))
        static_sizer.Add(self._base_status, 0)
        self._base_status.Bind(wx.EVT_LEFT_DOWN, self.on_base_status_clicked)
        
        # arm status
        self._arm_status = youbotStatusControl(self, wx.ID_ANY, youbot_icons_path, "arm", True)
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Stale"))
        static_sizer.Add(self._arm_status, 0)
        self._arm_status.Bind(wx.EVT_LEFT_DOWN, self.on_arm_status_clicked)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Drv"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # driver status
        self._driver_status = youbotStatusControl(self, wx.ID_ANY, youbot_icons_path, "driver", True)
        self._driver_status.SetToolTip(wx.ToolTip("Driver: Stale"))
        static_sizer.Add(self._driver_status, 0)
        self._driver_status.Bind(wx.EVT_BUTTON, self.on_driver_status_clicked)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Battery State
        self._power_state_ctrl = PowerStateControl(self, wx.ID_ANY, icons_path)
        self._power_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        static_sizer.Add(self._power_state_ctrl, 1, wx.EXPAND)
        
        self._config = wx.Config("youbot_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)
    
        self._dashboard_agg_sub = rospy.Subscriber('dashboard_agg', DashboardState, self.dashboard_callback)
        
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        
    def __del__(self):
        self._dashboard_agg_sub.unregister()
        
    def on_timer(self, evt):
#      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      level = self._diagnostics_frame.get_top_level_state()
      if (level == -1 or level == 3):
        if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))
        
      self.update_rosout()
      
      if (rospy.get_time() - self._last_dashboard_message_time > 5.0):
          self._power_state_ctrl.set_stale()
          self._arm_status.set_stale()
          self._base_status.set_stale()
          self._driver_status.set_stale()
                    
          ctrls = [self._power_state_ctrl, self._arm_status, self._base_status, self._driver_status]
          for ctrl in ctrls:
              ctrl.SetToolTip(wx.ToolTip("No message received on dashboard_agg in the last 5 seconds"))
        
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()
      
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
    def on_base_status_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_base_switch_on, menu.Append(wx.ID_ANY, "Enable Base Motors"))
      menu.Bind(wx.EVT_MENU, self.on_base_switch_off, menu.Append(wx.ID_ANY, "Disable Base Motors"))
      self._base_status.toggle(True)
      self.PopupMenu(menu)
      self._base_status.toggle(False)
      
    def on_base_switch_on(self, evt):
      # if any of the breakers is not enabled ask if they'd like to enable them
      if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
          if (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_STANDBY):
              switch_on_base = rospy.ServiceProxy("/base/switchOnMotors", std_srvs.srv.Empty)
       
              try:
                  switch_on_base()
              except rospy.ServiceException, e:
                  wx.MessageBox("Failed to switch ON base motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
          elif (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_ENABLED):
              wx.MessageBox("Base motors are already switched ON", "Error", wx.OK|wx.ICON_ERROR)
          elif (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_DISABLED):
              wx.MessageBox("Base is not connected", "Error", wx.OK|wx.ICON_ERROR)
              
    def on_base_switch_off(self, evt):
      # if any of the breakers is not enabled ask if they'd like to enable them
      if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
          if (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_ENABLED):
              switch_off_base = rospy.ServiceProxy("/base/switchOffMotors", std_srvs.srv.Empty)
       
              try:
                  switch_off_base()
              except rospy.ServiceException, e:
                  wx.MessageBox("Failed to switch OFF base motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
          elif (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_STANDBY):
              wx.MessageBox("Base motors are already switched OFF", "Error", wx.OK|wx.ICON_ERROR)
          elif (self._dashboard_message.power_board_state.circuit_state[0] == PowerBoardState.STATE_DISABLED):
              wx.MessageBox("Base is not connected", "Error", wx.OK|wx.ICON_ERROR)
      
    def on_arm_status_clicked(self, evt):
      menu = wx.Menu()
      menu.Bind(wx.EVT_MENU, self.on_arm_switch_on, menu.Append(wx.ID_ANY, "Enable Arm Motors"))
      menu.Bind(wx.EVT_MENU, self.on_arm_switch_off, menu.Append(wx.ID_ANY, "Disable Arm Motors"))
      self._arm_status.toggle(True)
      self.PopupMenu(menu)
      self._arm_status.toggle(False)
      
    def on_arm_switch_on(self, evt):
      # if any of the breakers is not enabled ask if they'd like to enable them
      if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
          if (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_STANDBY):
              switch_on_arm = rospy.ServiceProxy("/arm_1/switchOnMotors", std_srvs.srv.Empty)
       
              try:
                  switch_on_arm()
              except rospy.ServiceException, e:
                  wx.MessageBox("Failed to switch ON arm motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
          elif (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_ENABLED):
              wx.MessageBox("Arm motors are already switched ON", "Error", wx.OK|wx.ICON_ERROR)
          elif (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_DISABLED):
              wx.MessageBox("Arm is not connected", "Error", wx.OK|wx.ICON_ERROR)
              
    def on_arm_switch_off(self, evt):
      # if any of the breakers is not enabled ask if they'd like to enable them
      if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
          if (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_ENABLED):
              switch_off_arm = rospy.ServiceProxy("/arm_1/switchOffMotors", std_srvs.srv.Empty)
       
              try:
                  switch_off_arm()
              except rospy.ServiceException, e:
                  wx.MessageBox("Failed to switch OFF arm motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
          elif (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_STANDBY):
              wx.MessageBox("Arm motors are already switched OFF", "Error", wx.OK|wx.ICON_ERROR)
          elif (self._dashboard_message.power_board_state.circuit_state[1] == PowerBoardState.STATE_DISABLED):
              wx.MessageBox("Arm is not connected", "Error", wx.OK|wx.ICON_ERROR)

    def on_driver_status_clicked(self, evt):
        if (self._dashboard_message is not None):        
            reconnect = rospy.ServiceProxy("/reconnect", std_srvs.srv.Empty)

            try:
                reconnect()
            except rospy.ServiceException, e:
                wx.MessageBox("Failed to reconnect the driver: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)

            
    def dashboard_callback(self, msg):
      wx.CallAfter(self.new_dashboard_message, msg)
      
    def new_dashboard_message(self, msg):
      self._dashboard_message = msg
      self._last_dashboard_message_time = rospy.get_time()
      
      if (msg.power_state_valid):
        self._power_state_ctrl.set_power_state(msg.power_state)
      else:
        self._power_state_ctrl.set_stale()

      if (msg.power_board_state_valid and msg.power_board_state.circuit_state[0] == PowerBoardState.STATE_ENABLED):
        self._base_status.set_ok()
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: Switched ON"))
      elif (msg.power_board_state_valid and msg.power_board_state.circuit_state[0] == PowerBoardState.STATE_STANDBY):
        self._base_status.set_warn()
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: Switched OFF"))
      elif (msg.power_board_state_valid and msg.power_board_state.circuit_state[0] == PowerBoardState.STATE_DISABLED):
        self._base_status.set_error()
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: not connected"))
      else:
        self._base_status.set_stale()
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: stale"))

      if (msg.power_board_state_valid and msg.power_board_state.circuit_state[1] == PowerBoardState.STATE_ENABLED):
        self._arm_status.set_ok()
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Switched ON"))
      elif (msg.power_board_state_valid and msg.power_board_state.circuit_state[1] == PowerBoardState.STATE_STANDBY):
        self._arm_status.set_warn()
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Switched OFF"))
      elif (msg.power_board_state_valid and msg.power_board_state.circuit_state[1] == PowerBoardState.STATE_DISABLED):
        self._arm_status.set_error()
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: not connected"))
      else:
        self._arm_status.set_stale()
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: stale"))
          
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      
