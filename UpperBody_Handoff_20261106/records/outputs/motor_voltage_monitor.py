#!/usr/bin/env python3
"""Read-only ROS voltage dashboard. No motor publishers, service calls or CAN access."""
import argparse
import csv
from datetime import datetime, timezone
import logging
from logging.handlers import RotatingFileHandler
import queue
import threading
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, QUrl
from PyQt5.QtGui import QColor, QPainter, QPen, QDesktopServices
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget, QComboBox, QHBoxLayout, QFrame, QPushButton, QDialog
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

IDS = [1,3,5,7,9,2,4,6,8,10]
COLORS = ['#60a5fa','#fb923c','#4ade80','#c084fc','#f87171','#22d3ee','#facc15','#e879f9','#a3e635','#e2e8f0']
NAMES = {'shoulder_pitch':'어깨 앞뒤','shoulder_roll':'어깨 위아래','shoulder_yaw':'어깨 회전','elbow_pitch':'팔꿈치 굽힘','elbow_roll':'팔꿈치 회전'}


# These are FOC q-axis/phase currents, NOT DC supply input current.
METRICS = [('bus_voltage_v','전압 V'),('iq_measured_a','Iq 실측 A'),
           ('iq_setpoint_a','Iq 명령 A'),('id_measured_a','Id 실측 A'),
           ('torque_measured_nm','토크 실측 N·m'),('torque_setpoint_nm','토크 명령 N·m'),
           ('ia_measured_a','상전류 Ia A'),('ib_measured_a','상전류 Ib A'),('ic_measured_a','상전류 Ic A')]
TEMPERATURE_KEYS = ('board_temperature_c','motor_temperature_c','temperature_c','mosfet_temperature_c')


def metric(joint,key,elapsed=0):
    b=joint.get('board',{});a=joint.get('board_age_s',{})
    value=b.get(key);age=a.get(key)
    valid=isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    age=age+elapsed if isinstance(age,(int,float)) and not isinstance(age,bool) and math.isfinite(age) and age>=0 else None
    return (float(value) if valid else None),age


def temperature(joint,elapsed=0):
    for key in TEMPERATURE_KEYS:
        if key in joint.get('board',{}):return key,*metric(joint,key,elapsed)
    return None,None,None


class Recorder:
    """Bounded nonblocking queue; full status JSONL plus selected metric CSV."""
    def __init__(self,root):
        self.directory=Path(root)/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True,exist_ok=True)
        self.q=queue.Queue(maxsize=1000);self.dropped=0;self.error='';self.count=0;self.part=0
        self.handler=RotatingFileHandler(self.directory/'status.jsonl',maxBytes=64*1024*1024,backupCount=8,encoding='utf-8')
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.csvfile=(self.directory/'motors.csv').open('w',newline='',encoding='utf-8')
        self.writer=csv.writer(self.csvfile)
        self.writer.writerow(['received_utc','dry_run','motor_id','joint','live','fault','warning','mode','pdo_age_s','measured_deg','command_deg','goal_deg','velocity_dps']+[x for k,_ in METRICS for x in (k,k+'_age_s')]+['temperature_key','temperature_c','temperature_age_s'])
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def submit(self,status):
        item={'received_utc':datetime.now(timezone.utc).isoformat(),'received_monotonic_s':time.monotonic(),'status':status}
        try:self.q.put_nowait(item)
        except queue.Full:self.dropped+=1

    def run(self):
        last_flush=time.monotonic()
        while True:
            item=self.q.get()
            try:
                if item is None:return
                # handleError normally hides file errors; explicit stream writes let the UI report them.
                line=json.dumps(item,ensure_ascii=False,allow_nan=False)
                record=logging.LogRecord('telemetry',logging.INFO,'',0,line,(),None)
                if self.handler.shouldRollover(record):
                    self.handler.stream.flush();self.handler.stream.close();self.part+=1
                    Path(self.handler.baseFilename).rename(self.directory/f'status_part_{self.part:06d}.jsonl')
                    self.handler.stream=self.handler._open()
                self.handler.stream.write(line+'\n')
                s=item['status']
                for name,j in s.get('joints',{}).items():
                    if not isinstance(j,dict):continue
                    row=[item['received_utc'],s.get('dry_run'),j.get('device_id'),name]+[j.get(k) for k in ('live','fault','warning','board_mode','pdo_age_s','measured_deg','command_deg','goal_deg','velocity_dps')]
                    for key,_ in METRICS:row.extend(metric(j,key))
                    row.extend(temperature(j));self.writer.writerow(row)
                self.count+=1
                if time.monotonic()-last_flush>1:
                    self.handler.flush();self.csvfile.flush();last_flush=time.monotonic()
            except Exception as exc:self.error=str(exc)
            finally:self.q.task_done()

    def close(self):
        self.q.put(None);self.thread.join(timeout=5)
        if not self.thread.is_alive():self.handler.close();self.csvfile.close()


def age_now(joint, elapsed):
    value = joint.get('board_age_s',{}).get('bus_voltage_v')
    return value + elapsed if isinstance(value,(int,float)) and math.isfinite(value) and value >= 0 else None


def voltage(joint):
    value = joint.get('board',{}).get('bus_voltage_v')
    return float(value) if isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) else None



def display_status(status):
    """Adapt SDK 0.2 flat health fields for display; never invent missing current/temperature."""
    result=dict(status);result['joints']={}
    aliases={'bus_voltage_v':'bus_voltage_v','mode':'hardware_mode','error':'hardware_error',
             'current_limit_a':'current_limit_a','torque_constant_nm_per_a':'torque_constant_nm_per_a',
             'torque_limit_nm':'configured_torque_limit','firmware_version':'firmware_version',
             'firmware_gear_ratio':'firmware_gear_ratio'}
    for name,j in status.get('joints',{}).items():
        if not isinstance(j,dict):continue
        mapped=dict(j)
        if 'board' not in j and ('feedback_age_s' in j or 'hardware_mode_name' in j):
            mapped['board']={key:j[source] for key,source in aliases.items() if source in j}
            mapped['board_age_s']={'bus_voltage_v':j.get('voltage_age_s'),'mode':j.get('mode_age_s'),'error':j.get('error_age_s')}
            mapped['pdo_age_s']=j.get('feedback_age_s')
            mapped['board_mode']=j.get('hardware_mode_name','미수신')
            mapped['warning']=j.get('health_warning','')
        result['joints'][name]=mapped
    return result


class Reader(Node):
    def __init__(self,recorder=None):
        self.recorder=recorder
        super().__init__('motor_voltage_readonly_monitor', start_parameter_services=False)
        self.status = None
        self.received = None
        self.bad_messages = 0
        self.publisher_count = 0
        self.graph_checked = 0
        self.series = {i:deque(maxlen=1800) for i in IDS}
        self.last_sample = {}
        self.histories={k:{i:deque(maxlen=1800) for i in IDS} for k,_ in METRICS}
        self.sample_times={}
        self.create_subscription(String, '/arm_controller/status', self.on_status, 5)

    def on_status(self, message):
        now=time.monotonic()
        try:
            status=json.loads(message.data)
            if not isinstance(status,dict) or not isinstance(status.get('joints'),dict):
                raise ValueError('invalid status schema')
        except (ValueError,TypeError):
            self.bad_messages += 1
            return
        if getattr(self,"recorder",None):self.recorder.submit(status)
        status=display_status(status)
        self.status,self.received=status,now
        for j in status['joints'].values():
            if not isinstance(j,dict):continue
            did=j.get('device_id');v=voltage(j);age=age_now(j,0)
            if hasattr(self,'histories') and did in IDS and not status.get('dry_run'):
                for key,_ in METRICS:
                    value,metric_age=metric(j,key)
                    if value is None or metric_age is None or metric_age>2:continue
                    sample=now-metric_age;token=(did,key)
                    if sample-self.sample_times.get(token,-1e9)>.04:
                        self.histories[key][did].append((sample,value));self.sample_times[token]=sample
            if did not in self.series or v is None or age is None or age>2 or status.get('dry_run'):
                continue
            estimated_sample=now-age
            if estimated_sample-self.last_sample.get(did,-1e9)>.04:
                self.series[did].append((estimated_sample,v))
                self.last_sample[did]=estimated_sample


BLUE = '#174cff'
ORANGE = '#c45b00'
INK = '#101828'
MUTED = '#475467'


def joint_label(did):
    names=['어깨 앞뒤','어깨 위아래','어깨 회전','팔꿈치 굽힘','팔꿈치 회전']
    index=(did-1)//2 if did%2 else (did-2)//2
    return names[index] if 0<=index<5 else '미확인'


def is_fresh(value,age,elapsed,dry_run=False):
    return value is not None and age is not None and age<=2 and elapsed is not None and elapsed<=2 and not dry_run


class TrendGraph(QWidget):
    def __init__(self,reader):
        super().__init__();self.reader=reader;self.selected=3;self.voltage_mode=False
        self.setMinimumHeight(150)

    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#ffffff'))
        r=QRectF(56,16,max(80,self.width()-120),max(70,self.height()-55))
        now=time.monotonic();keys=['bus_voltage_v'] if self.voltage_mode else ['iq_measured_a','torque_measured_nm']
        series={k:[(t,v) for t,v in self.reader.histories[k][self.selected] if now-60<=t<=now] for k in keys}
        def limits(key):
            values=[v for _,v in series[key]]
            if key=='bus_voltage_v':lo=min([20]+values);hi=max([28]+values)
            else:lo=min([0]+values);hi=max([1 if 'torque' in key else 3]+values)
            pad=(hi-lo)*.08
            return lo-pad,hi+pad
        ranges={k:limits(k) for k in keys}
        colors={keys[0]:BLUE}
        if len(keys)>1:colors[keys[1]]=ORANGE
        for i in range(6):
            y=r.bottom()-r.height()*i/5
            p.setPen(QPen(QColor('#e4e7ec'),1));p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y))
            for k in keys:
                lo,hi=ranges[k];value=lo+(hi-lo)*i/5
                p.setPen(QColor(colors[k]));x=0 if k==keys[0] else int(r.right()+8)
                p.drawText(QRectF(x,y-10,50,20),Qt.AlignRight if k==keys[0] else Qt.AlignLeft,f'{value:.2f}')
        for i in range(7):
            x=r.left()+r.width()*i/6
            p.setPen(QPen(QColor('#e4e7ec'),1));p.drawLine(QPointF(x,r.top()),QPointF(x,r.bottom()))
            p.setPen(QColor(MUTED));p.drawText(QRectF(x-28,r.bottom()+8,56,22),Qt.AlignCenter,'현재' if i==6 else f'-{60-i*10}s')
        p.save();p.setClipRect(r)
        for k in keys:
            lo,hi=ranges[k]
            def point(t,v):return QPointF(r.left()+r.width()*(t-now+60)/60,r.bottom()-r.height()*(v-lo)/(hi-lo))
            p.setPen(QPen(QColor(colors[k]),2.3))
            samples=series[k]
            for a,b in zip(samples,samples[1:]):
                if b[0]-a[0]<3:p.drawLine(point(*a),point(*b))
            if len(samples)==1:p.drawEllipse(point(*samples[0]),2,2)
        p.restore()
        if not any(series.values()):
            p.fillRect(QRectF(r.left()+25,r.center().y()-22,r.width()-50,44),QColor('#ffffff'))
            p.setPen(QColor(MUTED));p.drawText(r,Qt.AlignCenter,'최근 60초의 유효한 측정값이 없습니다')


class Dashboard(QWidget):
    def __init__(self,reader,state_file=None,snapshot=None):
        super().__init__();self.reader=reader;self.state_file=state_file;self.snapshot=snapshot
        self.selected=3;self.show_all=False;self.captured=False;self.last_draw=0
        self.setWindowTitle('팔 모터 진단 · Berkeley Humanoid Lite')
        self.resize(1480,940);self.setMinimumSize(1100,800)
        self.setStyleSheet('''
            QWidget {background:#f7f9fc;color:#101828;font-family:"Noto Sans CJK KR","DejaVu Sans";font-size:14px;}
            QFrame#panel {background:white;border:1px solid #cbd2df;border-radius:7px;}
            QFrame#panel QLabel, QFrame#panel QWidget {background:transparent;}
            QLabel#brand {font-size:25px;font-weight:800;}
            QPushButton {background:#fff;border:1px solid #b6bfce;border-radius:5px;padding:8px 14px;color:#101828;font-weight:600;}
            QPushButton:hover {background:#edf2ff;border-color:#174cff;}
            QPushButton#motor {text-align:left;border:0;border-bottom:1px solid #d0d5dd;border-radius:0;padding:17px 16px;font-size:17px;}
            QPushButton#motor:checked {background:#e4edff;color:#101828;border-left:5px solid #174cff;font-weight:800;}
            QTableWidget {background:#fff;alternate-background-color:#f4f6fa;border:0;gridline-color:#d0d5dd;color:#101828;selection-background-color:#e0eaff;selection-color:#101828;}
            QHeaderView::section {background:#e8edf5;color:#101828;border:0;border-bottom:1px solid #cbd2df;padding:8px;font-weight:700;}
            QComboBox {background:white;color:#101828;border:1px solid #cbd2df;padding:4px 8px;border-radius:4px;}
            QDialog {background:#f7f9fc;}
        ''')
        root=QVBoxLayout(self);root.setContentsMargins(18,12,18,12);root.setSpacing(12)
        header=QHBoxLayout();brand=QLabel('팔 모터 진단');brand.setObjectName('brand');header.addWidget(brand)
        sub=QLabel('  |  Berkeley Humanoid Lite');sub.setStyleSheet('font-size:18px;color:#475467;');header.addWidget(sub);header.addStretch()
        self.source_badge=QLabel('상태 대기');header.addWidget(self.source_badge)
        readonly=QLabel('읽기 전용');readonly.setStyleSheet('background:#e4e7ec;padding:7px 12px;border-radius:5px;font-weight:700;');header.addWidget(readonly)
        self.temp_badge=QLabel('온도 미수신');header.addWidget(self.temp_badge);root.addLayout(header)
        body=QHBoxLayout();body.setSpacing(16);root.addLayout(body,1)
        rail=self.panel();rail.setFixedWidth(210);rail_layout=QVBoxLayout(rail);rail_layout.setContentsMargins(0,4,0,10);rail_layout.setSpacing(0)
        self.motor_buttons={}
        for did in IDS:
            b=QPushButton(f'ID {did}   {joint_label(did)}');b.setObjectName('motor');b.setCheckable(True)
            b.setChecked(did==self.selected);b.setVisible(did%2==1)
            b.clicked.connect(lambda checked=False,i=did:self.select_motor(i))
            rail_layout.addWidget(b);self.motor_buttons[did]=b
        rail_layout.addStretch();self.scope=QPushButton('양팔 전체 보기');self.scope.clicked.connect(self.toggle_scope);rail_layout.addWidget(self.scope)
        body.addWidget(rail)
        main=QVBoxLayout();main.setSpacing(12);body.addLayout(main,1)
        self.motor_title=QLabel();self.motor_title.setStyleSheet('font-size:28px;font-weight:800;');main.addWidget(self.motor_title)
        self.banner=QLabel('서버 상태를 기다리는 중');self.banner.setWordWrap(True);main.addWidget(self.banner)
        self.cards_panel=self.panel();cards_layout=QHBoxLayout(self.cards_panel);cards_layout.setContentsMargins(18,14,18,14);cards_layout.setSpacing(14)
        self.cards={}
        for index,(key,title,unit) in enumerate([('bus_voltage_v','전압','V'),('iq_measured_a','전류 Iq','A'),('torque_measured_nm','토크','N·m'),('temperature','온도','°C')]):
            column=QVBoxLayout();column.setSpacing(5)
            label=QLabel(title);label.setStyleSheet('font-size:18px;font-weight:700;');column.addWidget(label)
            value=QLabel('— '+unit);value.setStyleSheet('font-size:35px;font-weight:800;');column.addWidget(value)
            age=QLabel('센서값 미수신' if key=='temperature' else '측정 대기');age.setWordWrap(True);age.setStyleSheet('font-size:13px;color:#475467;');column.addWidget(age)
            cards_layout.addLayout(column,1);self.cards[key]=(value,age,unit)
            if index<3:
                line=QFrame();line.setFrameShape(QFrame.VLine);line.setStyleSheet('background:#d0d5dd;max-width:1px;');cards_layout.addWidget(line)
        main.addWidget(self.cards_panel)
        mid=QHBoxLayout();mid.setSpacing(14);main.addLayout(mid,1)
        chart=self.panel();chart_layout=QVBoxLayout(chart);chart_layout.setContentsMargins(16,10,16,8)
        graphhead=QHBoxLayout();self.graph_title=QLabel('전류 · 토크 추이');self.graph_title.setStyleSheet('font-size:19px;font-weight:700;');graphhead.addWidget(self.graph_title)
        graphhead.addStretch();self.chart_choice=QComboBox();self.chart_choice.addItems(['전류 · 토크','전압']);self.chart_choice.currentIndexChanged.connect(self.change_chart);graphhead.addWidget(self.chart_choice)
        chart_layout.addLayout(graphhead);self.legend=QLabel();chart_layout.addWidget(self.legend)
        self.graph=TrendGraph(reader);chart_layout.addWidget(self.graph,1);mid.addWidget(chart,3)
        position=self.panel();position.setMinimumWidth(270);position.setMaximumWidth(340)
        pos_layout=QVBoxLayout(position);pos_layout.setContentsMargins(16,10,16,8);pos_layout.setSpacing(3)
        label=QLabel('위치 추종');label.setStyleSheet('font-size:19px;font-weight:700;');pos_layout.addWidget(label)
        self.position_values={}
        for key,title in [('actual','실제'),('target','목표'),('error','오차'),('mode','상태'),('limit','토크 제한')]:
            row=QHBoxLayout();name=QLabel(title);row.addWidget(name);row.addStretch();value=QLabel('—');value.setStyleSheet('font-size:20px;font-weight:700;');row.addWidget(value)
            pos_layout.addLayout(row);self.position_values[key]=value
        self.position_age=QLabel('측정 대기');self.position_age.setWordWrap(True);self.position_age.setStyleSheet('color:#475467;font-size:12px;');pos_layout.addWidget(self.position_age)
        mid.addWidget(position,1)
        bottom=self.panel();bottom_layout=QVBoxLayout(bottom);bottom_layout.setContentsMargins(12,10,12,8)
        tablehead=QHBoxLayout();title=QLabel('모터 전체 상태');title.setStyleSheet('font-size:20px;font-weight:700;');tablehead.addWidget(title);tablehead.addStretch()
        hint=QLabel('행 선택 → 해당 모터 상세');hint.setStyleSheet('color:#475467;font-size:12px;');tablehead.addWidget(hint);bottom_layout.addLayout(tablehead)
        self.table=QTableWidget(5,7);self.table.setHorizontalHeaderLabels(['모터','전압 V','Iq A','토크 N·m','온도 °C','측정 경과','상태'])
        self.table.verticalHeader().hide();self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers);self.table.setSelectionBehavior(QTableWidget.SelectRows);self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setFixedHeight(225);self.table.verticalHeader().setMinimumSectionSize(25);self.table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch);self.table.cellClicked.connect(self.table_select)
        bottom_layout.addWidget(self.table);main.addWidget(bottom)
        footer=self.panel();foot=QHBoxLayout(footer);foot.setContentsMargins(14,8,14,8)
        self.loglabel=QLabel('로그 준비 중');foot.addWidget(self.loglabel,1)
        self.open_log=QPushButton('로그 폴더 열기');self.open_log.clicked.connect(self.open_logs);foot.addWidget(self.open_log)
        self.open_details=QPushButton('전체 측정 항목');self.open_details.clicked.connect(self.show_details);foot.addWidget(self.open_details)
        root.addWidget(footer)
        note=QLabel('Iq는 모터 q축 전류입니다. 전원 입력 전류와 다릅니다.  |  2초 초과 측정은 과거값으로 표시  |  모터 명령 전송 없음')
        note.setStyleSheet('color:#475467;font-size:12px;');root.addWidget(note)
        self.detail_dialog=QDialog(self);self.detail_dialog.setWindowTitle('전체 수신 항목 · 개별 측정 시간');self.detail_dialog.resize(1250,780)
        detail_layout=QVBoxLayout(self.detail_dialog)
        detail_layout.addWidget(QLabel('실제 수신 필드 전체 · 보드 값과 측정 경과 시간은 항목별로 다릅니다.'))
        self.details=QTableWidget(0,7);self.details.setHorizontalHeaderLabels(['모터','구분','항목 / 단위','값','측정 경과 s','신선도','관절'])
        self.details.setEditTriggers(QTableWidget.NoEditTriggers);self.details.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.details.horizontalHeader().setStretchLastSection(True);detail_layout.addWidget(self.details)
        self.change_chart(0)
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(50)

    @staticmethod
    def panel():
        f=QFrame();f.setObjectName('panel');return f

    def select_motor(self,did):
        self.selected=did;self.graph.selected=did
        for i,b in self.motor_buttons.items():b.setChecked(i==did)
        self.last_draw=0;self.tick()

    def toggle_scope(self):
        self.show_all=not self.show_all
        for i,b in self.motor_buttons.items():b.setVisible(self.show_all or i%2==1)
        self.scope.setText('왼팔만 보기' if self.show_all else '양팔 전체 보기')
        if not self.show_all and self.selected%2==0:self.select_motor(3)
        self.last_draw=0;self.tick()

    def table_select(self,row,col):
        self.select_motor((IDS if self.show_all else IDS[:5])[row])

    def change_chart(self,index):
        self.graph.voltage_mode=index==1
        self.graph_title.setText('전압 추이' if index==1 else '전류 · 토크 추이')
        self.legend.setText('<span style="color:#174cff">━ 전압 [V]</span>' if index==1 else '<span style="color:#174cff">━ 전류 Iq [A]</span>　 <span style="color:#a34700">━ 토크 [N·m]</span>')
        self.graph.update()

    def open_logs(self):
        recorder=getattr(self.reader,'recorder',None)
        if recorder:QDesktopServices.openUrl(QUrl.fromLocalFile(str(recorder.directory)))

    def show_details(self):
        elapsed=time.monotonic()-self.reader.received if self.reader.received is not None else None
        self.update_details(self.reader.status or {},elapsed);self.detail_dialog.show();self.detail_dialog.raise_()

    def tick(self):
        if not rclpy.ok():self.timer.stop();self.close();QApplication.quit();return
        for _ in range(3):rclpy.spin_once(self.reader,timeout_sec=0)
        now=time.monotonic()
        if now-self.last_draw<.2:return
        self.last_draw=now;s=self.reader.status or {};elapsed=now-self.reader.received if self.reader.received is not None else None
        if now-self.reader.graph_checked>1:
            self.reader.graph_checked=now;self.reader.publisher_count=len(self.reader.get_publishers_info_by_topic('/arm_controller/status'))
        peers=self.reader.publisher_count;dry=s.get('dry_run',False)
        byid={j.get('device_id'):j for j in s.get('joints',{}).values() if isinstance(j,dict)}
        selected=byid.get(self.selected,{})
        self.motor_title.setText(f'CAN ID {self.selected}   |   '+('왼팔' if self.selected%2 else '오른팔')+' · '+joint_label(self.selected))
        tk,tv,ta=temperature(selected,elapsed or 0)
        for key,(label,age_label,unit) in self.cards.items():
            value,age=(tv,ta) if key=='temperature' else metric(selected,key,elapsed or 0)
            fresh=is_fresh(value,age,elapsed,dry)
            label.setText(('—' if value is None else f'{value:.2f}')+' '+unit)
            label.setStyleSheet('font-size:35px;font-weight:800;color:'+('#101828' if fresh else '#667085')+';')
            age_label.setText(('센서값 미수신' if key=='temperature' else '측정값 미수신') if value is None else ('연습 모드' if dry else (('드라이버 보고' if key=='bus_voltage_v' else '실측') if fresh else '과거값 / 시간 미확인')+(' · '+f'{age:.1f}초 전' if age is not None else '')))
            age_label.setStyleSheet('font-size:13px;color:'+('#475467' if fresh else '#92400e')+';')
        self.temp_badge.setText('온도 수신' if is_fresh(tv,ta,elapsed,dry) else '온도 미수신' if tv is None else '온도 과거값')
        self.temp_badge.setStyleSheet('background:#fff0d9;color:#7a3600;padding:7px 12px;border-radius:5px;font-weight:700;')
        topic_live=elapsed is not None and elapsed<=2
        self.source_badge.setText(f'ROS {os.environ.get("ROS_DOMAIN_ID","0")} · '+('연습 모드' if dry else '상태 수신 중' if topic_live else '통신 대기 / 지연'))
        self.source_badge.setStyleSheet('font-size:13px;color:#475467;padding:6px;')
        fault=selected.get('fault');warning=selected.get('warning')
        if peers>1:message='상태 발행자 중복 · '+str(peers)+'개 · 데이터 출처 확인 필요'
        elif not topic_live:message='서버 상태 수신 대기' if elapsed is None else f'통신 지연 · 마지막 수신 {elapsed:.1f}초 전'
        elif dry:message='연습 모드 · 실제 모터 측정값으로 판단하지 마세요'
        elif fault:message='통신 / 드라이버 오류: '+str(fault)
        elif warning:message='조회 경고: '+str(warning)
        else:message='읽기 전용 모니터링 · 항목별 측정 경과 시간을 확인하세요'
        self.banner.setText(message);self.banner.setToolTip(str(warning or fault or ''))
        self.banner.setStyleSheet('font-size:13px;color:'+('#9b1c1c' if fault or peers>1 else '#475467')+';')
        actual=selected.get('measured_deg');goal=selected.get('goal_deg');pdo=selected.get('pdo_age_s')
        pa=pdo+(elapsed or 0) if isinstance(pdo,(int,float)) else None
        pos_fresh=is_fresh(actual,pa,elapsed,dry)
        for key,value in [('actual',actual),('target',goal),('error',(goal-actual) if isinstance(actual,(int,float)) and isinstance(goal,(int,float)) else None)]:
            self.position_values[key].setText(f'{value:.1f}°' if isinstance(value,(int,float)) else '—')
            self.position_values[key].setStyleSheet('font-size:20px;font-weight:700;color:'+('#101828' if pos_fresh else '#667085')+';')
        mode=selected.get('board_mode','미수신');ma=metric(selected,'mode',elapsed or 0)[1]
        self.position_values['mode'].setText(str(mode)+(' · 과거값' if ma is not None and ma>2 else ''))
        self.position_values['mode'].setStyleSheet('font-size:15px;font-weight:700;color:#174cff;')
        limit,limit_age=metric(selected,'torque_limit_nm',elapsed or 0)
        self.position_values['limit'].setText('—' if limit is None else f'{limit:.2f} N·m')
        self.position_values['limit'].setToolTip('설정 읽기값 · 측정 경과 '+(f'{limit_age:.1f}초' if limit_age is not None else '미확인'))
        self.position_age.setText('위치 '+('미수신' if pa is None else f'{pa:.1f}초 전')+(' · 과거값' if not pos_fresh else '')+'\nMODE·제한값은 마지막 조회값')
        ids=IDS if self.show_all else IDS[:5];self.table.setRowCount(len(ids));display_rows=[]
        for row,did in enumerate(ids):
            j=byid.get(did,{});v,age=metric(j,'bus_voltage_v',elapsed or 0);pdo=j.get('pdo_age_s')
            pdo_age=pdo+(elapsed or 0) if isinstance(pdo,(int,float)) else None
            board_old=not is_fresh(v,age,elapsed,dry);fault=j.get('fault');warn=j.get('warning')
            if dry:state='연습 모드'
            elif fault:state=str(fault)
            elif not j or pdo_age is None:state='미수신'
            elif not topic_live or pdo_age>2:state='통신 지연 / 과거값'
            elif warn:state='조회 경고'
            elif board_old:state='전압 과거값'
            else:state='수신 중'
            values=[f'ID {did}   {joint_label(did)}'];ages=[];fresh_flags=[]
            for key in ('bus_voltage_v','iq_measured_a','torque_measured_nm'):
                value,metric_age=metric(j,key,elapsed or 0);fresh=is_fresh(value,metric_age,elapsed,dry)
                values.append('—' if value is None else f'{value:.2f}'+(' [과거값]' if not fresh else ''));ages.append(metric_age);fresh_flags.append(fresh)
            _,temp,temp_age=temperature(j,elapsed or 0);values.append('—' if temp is None else f'{temp:.1f}');ages.append(temp_age);fresh_flags.append(is_fresh(temp,temp_age,elapsed,dry))
            values.append('미확인' if age is None else ('과거값 · ' if board_old else '')+f'{age:.1f}초');values.append(state)
            base='#e6edff' if did==self.selected else '#ffffff'
            for col,value in enumerate(values):
                item=QTableWidgetItem(value);item.setForeground(QColor('#475467' if 1<=col<=4 and not fresh_flags[col-1] else '#101828'))
                item.setBackground(QColor(base));item.setTextAlignment(Qt.AlignVCenter|(Qt.AlignLeft if col==0 else Qt.AlignCenter))
                if 1<=col<=4:item.setToolTip('항목 측정 경과: '+('미확인' if ages[col-1] is None else f'{ages[col-1]:.2f}초')+'\n온도 미수신은 0°C를 뜻하지 않습니다.')
                if col==6:
                    item.setToolTip(str(warn or fault or ''));item.setForeground(QColor('#9b1c1c' if fault else '#475467' if board_old else '#166534'))
                self.table.setItem(row,col,item)
            display_rows.append({'id':did,'voltage_v':v,'age_s':age,'stale':board_old,'state':state})
        recorder=getattr(self.reader,'recorder',None)
        if recorder:
            problem=bool(recorder.error or recorder.dropped)
            self.loglabel.setText(('저장 오류 / 누락' if problem else '● 로그 저장 중')+f'   |   JSONL + CSV   |   {recorder.count}회 · 누락 {recorder.dropped}회')
            self.loglabel.setStyleSheet('font-weight:600;color:'+('#9b1c1c' if problem else '#166534')+';')
            self.loglabel.setToolTip(str(recorder.directory)+('\n'+recorder.error if recorder.error else ''))
        if self.detail_dialog.isVisible():self.update_details(s,elapsed)
        self.graph.update()
        if self.state_file:
            Path(self.state_file).write_text(json.dumps({'pid':os.getpid(),'status_received':elapsed is not None,'publishers':peers,'dry_run':dry,'selected_id':self.selected,'log_directory':str(recorder.directory) if recorder else None,'log_count':recorder.count if recorder else 0,'log_error':recorder.error if recorder else '', 'rows':display_rows},ensure_ascii=False,indent=2))
        if self.snapshot and not self.captured and elapsed is not None:
            self.captured=True;QTimer.singleShot(600,lambda:self.grab().save(self.snapshot))

    def update_details(self,status,elapsed):
        rows=[]
        for name,j in sorted(status.get('joints',{}).items(),key=lambda kv:kv[1].get('device_id',999) if isinstance(kv[1],dict) else 999):
            if not isinstance(j,dict):continue
            did=j.get('device_id');ages=j.get('board_age_s',{})
            for group,fields in [('board',j.get('board',{})),('joint',{k:v for k,v in j.items() if k not in ('board','board_age_s')})]:
                for key,value in fields.items():
                    age=ages.get(key) if group=='board' else j.get('pdo_age_s') if key in ('measured_deg','velocity_dps') else 0
                    age=age+(elapsed or 0) if isinstance(age,(int,float)) and math.isfinite(age) and age>=0 else None
                    old=elapsed is None or elapsed>2 or age is None or age>2 or status.get('dry_run')
                    text=json.dumps(value,ensure_ascii=False) if isinstance(value,(dict,list)) else str(value)
                    rows.append([str(did),group,key,text,'미확인' if age is None else f'{age:.2f}','과거값 / 미확인' if old else '2초 이내',name])
        for key,value in status.items():
            if key!='joints':rows.append(['전체','controller',key,json.dumps(value,ensure_ascii=False),'—','수신 상태',''])
        self.details.setRowCount(len(rows))
        for row,values in enumerate(rows):
            for col,value in enumerate(values):
                value=str(value)
                item=QTableWidgetItem(value);item.setForeground(QColor('#111827'))
                item.setBackground(QColor('#e2e8f0' if values[5]=='과거값 / 미확인' else '#ffffff'))
                self.details.setItem(row,col,item)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir',default=str(Path.home()/'bhl_logs'/'pc_motor_monitor'));parser.add_argument('--state-file');parser.add_argument('--snapshot');parser.add_argument('--test-seconds',type=float)
    args,ros=parser.parse_known_args()
    app=QApplication(sys.argv[:1]);rclpy.init(args=ros);recorder=Recorder(args.log_dir);reader=Reader(recorder);window=Dashboard(reader,args.state_file,args.snapshot);window.show()
    if args.test_seconds:QTimer.singleShot(int(args.test_seconds*1000),app.quit)
    try:app.exec_()
    finally:reader.destroy_node();rclpy.try_shutdown();recorder.close()


if __name__=='__main__':main()
