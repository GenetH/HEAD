#!/usr/bin/env python

import unittest
import os
import sys
import atexit
import time
import subprocess

CWD = os.path.abspath(os.path.dirname(__file__))

os.environ['HR_CHARACTER_PATH'] = os.path.join(CWD, 'characters')
server_path = os.path.join(CWD, '../scripts')
PORT = '8002'
cmd = ['python', 'run_server.py', PORT]
proc = subprocess.Popen(cmd, cwd=server_path, preexec_fn=os.setsid)
def shutdown():
    if proc:
        os.killpg(proc.pid, 2)
atexit.register(shutdown)

class ChatbotTest(unittest.TestCase):

    def test_pyaiml(self):
        script = os.path.join(CWD, os.path.sep.join(
            ['..', 'src', 'chatbot', 'aiml', 'Kernel.py']))
        cmd = 'python '+script
        ret = os.system(cmd)
        self.assertTrue(ret==0)

    def test_prologue(self):
        from chatbot.client import Client
        cli = Client('test_client', 'AAAAB3NzaC')
        cli.do_port(PORT)
        while not cli.ping():
            time.sleep(1)
        cli.do_conn('localhost:'+PORT)
        cli.do_select('generic')
        ret, response = cli.ask('hello sophia')
        self.assertTrue(ret == 0)
        self.assertTrue(response.get('text') == 'Hi there from generic')

        cli.do_select('sophia')
        ret, response = cli.ask('hello sophia')
        self.assertTrue(ret == 0)
        self.assertTrue(response.get('text') == 'Hi there from sophia')

    def test_session_manager(self):
        from chatbot.server.session import SessionManager
        session_manager = SessionManager(False)
        sid = session_manager.start_session(user='test')
        session = session_manager.get_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        self.assertTrue(session.add("hi", "hi there"))
        self.assertIsNotNone(session.cache.last_time)

        session_manager.reset_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        session_manager.remove_session(sid)
        self.assertFalse(session.add("hi", "hi there"))
        session = session_manager.get_session(sid)
        self.assertIsNone(session)

    def test_session_manager_auto(self):
        import chatbot.server.config
        chatbot.server.config.SESSION_RESET_TIMEOUT = 1
        chatbot.server.config.SESSION_REMOVE_TIMEOUT = 2
        from chatbot.server.session import SessionManager
        reload(chatbot.server.session)

        session_manager = SessionManager(True)
        sid = session_manager.start_session(user='test')
        session = session_manager.get_session(sid)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        self.assertTrue(session.add("hi", "hi there"))
        self.assertIsNotNone(session.cache.last_time)

        time.sleep(1.5)
        self.assertIsNotNone(session)
        self.assertIsNone(session.cache.last_time)

        time.sleep(1)
        self.assertFalse(session.add("hi", "hi there"))
        session = session_manager.get_session(sid)
        self.assertIsNone(session)

if __name__ == '__main__':
    unittest.main()

