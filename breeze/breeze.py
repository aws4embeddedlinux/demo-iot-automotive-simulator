from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QFrame
import sys
import subprocess
import main_frame
import json
import logging
import boto3
from botocore.exceptions import ClientError

INITIAL_TIMER_DELAY = 5000
TIMER_DELAY = 5000

log = logging.getLogger('app_logger')
logging.basicConfig(format='[%(asctime)s %(levelname)s] %(message)s', level=logging.DEBUG)

fleetwise = boto3.client(
    'iotfleetwise'
    )

with open("./config.json", "r") as f:
    campaigns = json.load(f)

vehicle_id = campaigns["vehicleName"]

class VLine(QFrame):
    # a simple VLine, like the one you get from designer
    def __init__(self):
        super(VLine, self).__init__()
        self.setFrameShape(self.VLine|self.Sunken)

class BreezeApp(QtWidgets.QMainWindow, main_frame.Ui_MainWindow):
    def __init__(self, logger, parent=None):
        super(BreezeApp, self).__init__(parent)
        self.setupUi(self)
        self.labelVehicleID.setText("Vehicle ID :  " + vehicle_id)
        for e in campaigns["campaigns"]:
            self.comboBoxCampaigns.addItem(e["name"])
        self.btnDeploy.clicked.connect(self.btnDeploy_clicked)

        self.comboBoxCampaigns.currentIndexChanged.connect(self.comboBoxCampaigns_indexChanged)
        self.my_logger = logger
        self.my_logger.debug("Campaign selected: " + self.comboBoxCampaigns.currentText() + '  ' + str(0))
        self.timer=QTimer()
        self.timer.timeout.connect(self.timer_timeout)
        self.pixmap = QPixmap('./resources/aws_icon.png')

        self.labelIcon.setPixmap(self.pixmap)

        # Optional, resize label to image size
        self.labelIcon.resize(self.pixmap.width(), self.pixmap.height())
        self.statusBar.showMessage("Initialized. Please, deploy a campaign.")

        self.lbl1 = QLabel("Label: ")
        self.lbl1.setStyleSheet('border: 0; color:  blue;')
        self.lbl2 = QLabel("Data : ")
        self.lbl2.setStyleSheet('border: 0; color:  red;')

        self.statusBar.reformat()
        self.statusBar.setStyleSheet('border: 0; background-color: #FFF8DC;')
        self.statusBar.setStyleSheet("QStatusBar::item {border: none;}")

        self.statusBar.addPermanentWidget(VLine())    # <---
        self.statusBar.addPermanentWidget(self.lbl1)
        self.statusBar.addPermanentWidget(VLine())    # <---
        self.statusBar.addPermanentWidget(self.lbl2)

        self.lbl1.setText("  Vehicle Status : __________________   ")
        self.lbl2.setText("  Campaign Status : __________________   ")
        self.active_campaign = ""

    def timer_timeout(self):
        self.timer.stop()
        self.statusBar.showMessage("Checking campaign status... " + self.active_campaign)
        campaign_status = fleetwise.get_campaign(name=self.active_campaign)
        self.lbl2.setText("  Campaign Status : " + campaign_status["status"])
        if (campaign_status["status"] == "WAITING_FOR_APPROVAL"):
            print("Approving campaign : " + self.active_campaign )
            fleetwise.update_campaign(name=self.active_campaign, action="APPROVE")
        campaign_status = fleetwise.get_campaign(name=self.active_campaign)
        self.lbl2.setText("  Campaign Status : " + campaign_status["status"])
        self.statusBar.showMessage("Checking vehicle status... " + vehicle_id)
        vehicle_status = fleetwise.get_vehicle_status(vehicleName=vehicle_id)
        status = vehicle_status["campaigns"][0]["status"] if len(vehicle_status["campaigns"]) > 0 else ""
        self.lbl1.setText("  Vehicle Status : " + status)
        self.statusBar.showMessage("")
        self.timer.start(TIMER_DELAY)

    def comboBoxCampaigns_indexChanged(self, index):
        self.my_logger.debug("Campaign selected: " + self.comboBoxCampaigns.currentText() + '  ' + str(index))

    def btnDeploy_clicked(self):
        self.btnDeploy.setEnabled(False)
        self.comboBoxCampaigns.setEnabled(False)
        self.timer.stop()
        self.statusBar.showMessage("Deploying campaign...")
        self.my_logger.debug("Deploy Clicked")
        self.my_logger.debug("Delete all campaings first")
        for i in range(self.comboBoxCampaigns.count()):
            campaign = self.comboBoxCampaigns.itemText(i)
            self.my_logger.debug("<<<<< Delete campaign: " + campaign)
            self.statusBar.showMessage("Deleting a campaign: " + campaign)
            fleetwise.delete_campaign(name=campaign)
        campaign = self.comboBoxCampaigns.currentText()
        self.active_campaign = campaign
        self.lbl2.setText("  Campaign Status : __________________   ")
        for e in campaigns["campaigns"]:
            if e["name"] == self.comboBoxCampaigns.currentText():
                configJson = e["configJson"]
                print("Create campaign : " + e["name"] + " - " + configJson )
                self.statusBar.showMessage("Creating a campaign: " + campaign)
                process = subprocess.Popen(['aws', 'iotfleetwise', 'create-campaign', '--cli-input-json', configJson, '--no-cli-pager'  ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
                stdout,stderr = process.communicate()
                print(stdout)
                campaign_status = fleetwise.get_campaign(name=self.active_campaign)
                self.lbl2.setText("  Campaign Status : " + campaign_status["status"])
                self.statusBar.showMessage("Campaign created: " + campaign)
        self.btnDeploy.setEnabled(True)
        self.comboBoxCampaigns.setEnabled(True)
        self.timer.start(1000)


def main():
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    print('Screen: %s' % screen.name())
    size = screen.size()
    print('Size: %d x %d' % (size.width(), size.height()))
    rect = screen.availableGeometry()
    print('Available: %d x %d' % (rect.width(), rect.height()))
    form = BreezeApp(log)
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()


