import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib

from qtvcp import logger
LOG = logger.getLogger(__name__)
# Force the log level for this module
#LOG.setLevel(logger.DEBUG) # One of DEBUG, INFO, WARNING, ERROR, CRITICAL

# only import zmq if needed and present
def import_ZMQ():
    try:
        import zmq
    except:
        LOG.warning('Problem importing zmq - Is python3-zmq installed?')
        # nope no messaging for you
        return False
    else:
        import json
        # since we imported in a function we need to globalize
        global zmq
        global json
        # imports are good - go ahead and setup messaging
        return True

class ZMQSendReceive():
    def __init__(self, instance = None, topic = b""):
        self._inst = instance
        self.add_send_zmq = False
        self.add_receive_zmq = True

        self._zmq_sub_subscribe_name = topic
        self._zmq_sub_socket_address = "tcp://127.0.0.1:5690"
        self._zmq_pub_socket_address = "tcp://127.0.0.1:5690"

####################
# initialize
####################

    def openPublish(self):
        self.add_send_zmq = True
        self.init_zmq_publish()

    def openSubscribe(self):
        self.add_receive_zmq = True
        self.init_zmq_subscribe()

######################################
# subscribe
######################################

    def init_zmq_subscribe(self):
        if import_ZMQ():
            try:
                self._zmq_sub_context = zmq.Context()
                self._zmq_sub_sock = self._zmq_sub_context.socket(zmq.SUB)
                self._zmq_sub_sock.connect(self._zmq_sub_socket_address)
                self._zmq_sub_sock.setsockopt(zmq.SUBSCRIBE, self._zmq_sub_subscribe_name)
                zmq_fd = self._zmq_sub_sock.getsockopt(zmq.FD)
                channel = GLib.IOChannel.unix_new(zmq_fd)
                GLib.io_add_watch(channel, GLib.IO_IN, self.zmq_callback, self._zmq_sub_sock)

            except Exception as e:
                LOG.exception('zmq subscribe to message setup error: {}'.format(e))

    def zmq_callback(self, fd, condition, zmq_socket):
        while zmq_socket.getsockopt(zmq.EVENTS) & zmq.POLLIN:
            (channel, msg) = zmq_socket.recv_multipart()
            self.call_function(channel, msg)
        return True

    # convert message to a function call
    def call_function(self, topic, data):
        # convert from json object to python object
        y = json.loads(data)
        # get the function name
        function = y.get('FUNCTION')
        # get the arguments
        arguments = y.get('ARGS')
        if self._inst is None:
            print('{} Sent ZMQ Message:{} {}'.format(topic,function,arguments))
            return
        # call handler function with arguments
        try:
            # call self._inst (a function) with all arguments
            if callable(self._inst):
                self._inst(topic,function,arguments)
            else:
                # directly call the function of self._inst (a class instance)
                self._inst[function](arguments)
        except Exception as e:
            LOG.error('zmq message parcing error: {}'.format(e))
            LOG.error('{} {}'.format(function, arguments))

################################
# Publish
################################

    def init_zmq_publish(self):
        if import_ZMQ():
            try:
                self._zmq_pub_context = zmq.Context()
                self._zmq_pub_socket = self._zmq_pub_context.socket(zmq.PUB)
                self._zmq_pub_socket.bind(self._zmq_pub_socket_address)
            except Exception as e:
                LOG.error('zmq publish message setup error: {}'.format(e))

    def zmq_write_message(self, args,topic = 'gladevcp'):
        if self.add_send_zmq:
            try:
                message = json.dumps(args)
                LOG.debug('Sending ZMQ Message:{} {}'.format(topic,message))
                self._zmq_pub_socket.send_multipart(
                    [bytes(topic.encode('utf-8')),
                        bytes((message).encode('utf-8'))])
            except Exception as e:
                LOG.error('zmq message sending error: {}'.format(e))
        else:
            LOG.info('ZMQ Message not enabled. message:{} {}'.format(topic,args))

####################################
# Testing
####################################
if __name__ == "__main__":

    n = ZMQSendReceive()
    n.init_zmq_subscribe()

    # loop till exit
    try:
        GLib.MainLoop().run()
    except KeyboardInterrupt:
        raise SystemExit



