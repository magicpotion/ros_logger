from datetime import datetime as dt
from sqlalchemy import create_engine
from sqlalchemy import Column, Sequence
from sqlalchemy.orm import sessionmaker
from sqlalchemy.types import TEXT, TIMESTAMP, INT, VARCHAR
from sqlalchemy.dialects.postgresql import JSON
from sqlalchemy.ext.declarative import declarative_base

DB = 'hr'
HOST = '127.0.0.1'
# PORT = 5432
PORT = 5433
USER = 'hr'

db = create_engine(
    'postgresql://{USER}@{HOST}:{PORT}/{DB}'.format(**locals()),
    echo=False,
    connect_args={'connect_timeout': 5})
Base = declarative_base()
Session = sessionmaker(bind=db)
session = Session()


class LogDB(Base):
    __tablename__ = 'systemlog'
    id = Column(INT, Sequence('log_id_seq'), primary_key=True)
    datetime = Column(TIMESTAMP(timezone=False))
    source = Column(VARCHAR(160), nullable=False, default='')
    event = Column(VARCHAR(160), nullable=False, default='')
    info = Column(JSON, default='')

    def __repr__(self):
        return "<SystemStatus(source={}, datetime={}, info={})>".format(
            self.source, self.datetime, self.info)

    @staticmethod
    def insert(source, info, event='ros'):
        row = LogDB(
            source=source,
            event=event,
            info=info,
            datetime=dt.utcnow())
        session.add(row)
        session.commit()


class ChatLogDB(Base):
    __tablename__ = 'chatlog'
    id = Column(INT, Sequence('chatlog_id_seq'), primary_key=True)
    datetime = Column(TIMESTAMP(timezone=False))
    source = Column(VARCHAR(160), nullable=False, default='')
    event = Column(VARCHAR(160), nullable=False, default='')
    author = Column(TEXT(convert_unicode=True), default='')
    message = Column(TEXT(convert_unicode=True), default='')
    msgtype = Column(TEXT(convert_unicode=True), default='')

    def __repr__(self):
        return "<Chatlog(source={}, datetime={}, author={}, message={})>".format(
            self.source, self.datetime, self.author, self.message)

    @staticmethod
    def insert(source, author, message, msgtype, event=''):
        row = ChatLogDB(
            source=source,
            event=event,
            author=author,
            message=message,
            msgtype=msgtype,
            datetime=dt.utcnow())
        session.add(row)
        session.commit()


class SpeechInputDB(Base):
    __tablename__ = 'speech_input'
    id = Column(INT, Sequence('speech_input_id_seq'), primary_key=True)
    datetime = Column(TIMESTAMP(timezone=False))
    source = Column(VARCHAR(160), nullable=False, default='')
    message = Column(TEXT(convert_unicode=True), default='')
    confidence = Column(INT, default='')
    lang = Column(VARCHAR(convert_unicode=True, length=8), default='')
    audio_path = Column(TEXT(convert_unicode=True), default='')

    def __repr__(self):
        return "<SpeechInput(source={}, datetime={}, message={})>".format(
            self.source, self.datetime, self.message)

    @staticmethod
    def insert(source, message, confidence, lang, audio_path):
        row = SpeechInputDB(
            source=source,
            message=message,
            confidence=confidence,
            lang=lang,
            audio_path=audio_path,
            datetime=dt.utcnow())
        session.add(row)
        session.commit()


Base.metadata.create_all(db)
