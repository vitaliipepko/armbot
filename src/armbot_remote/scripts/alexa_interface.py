#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from armbot_remote.msg import ArmbotTaskAction, ArmbotTaskGoal
import rospy
import threading
import actionlib

threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()
client = actionlib.SimpleActionClient('task_server', ArmbotTaskAction)

app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, how can we help?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(
            False)

        goal = ArmbotTaskGoal(task_number=0)
        client.send_goal(goal)

        return handler_input.response_builder.response


class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Ok, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)

        goal = ArmbotTaskGoal(task_number=1)
        client.send_goal(goal)

        return handler_input.response_builder.response


class MoveIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("MoveForwardIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Ok, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Move", speech_text)).set_should_end_session(
            True)

        goal = ArmbotTaskGoal(task_number=2)
        client.send_goal(goal)

        return handler_input.response_builder.response


class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Ok, see you later"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)

        goal = ArmbotTaskGoal(task_number=3)
        client.send_goal(goal)

        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "Hi, I am ready"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
            
        goal = ArmbotTaskGoal(task_number=0)
        client.send_goal(goal)

        return handler_input.response_builder.response


class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):

        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(MoveIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())


skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.9ce59cda-1ebc-449c-a765-b76f8a4a4bae",
    app=app)


skill_adapter.register(app=app, route="/")


if __name__ == '__main__':
    app.run()