/*  MIT License

    Copyright (c) 2019 Niryo

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */

using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace NiryoOneClient
{
    /// <summary>
    /// A connection object allowing sending commands to the tcp server on a Niryo One robotic arm
    /// </summary>
    public class NiryoOneConnection : INiryoOneConnection
    {
        private readonly TextWriter _textWriter;
        private readonly int _numRetries;
        private readonly TextReader _textReader;

        /// <summary>
        /// Construct a connection object
        /// </summary>
        /// <param name="streamReader">A stream reader used for getting responses</param>
        /// <param name="streamWriter">A stream writer used for sending commands</param>
        /// <param name="numRetries">Number of retries for commands when Niryo reports robot is busy</param>
        public NiryoOneConnection(TextReader streamReader, TextWriter streamWriter, int numRetries = 2)
        {
            _textWriter = streamWriter;
            _numRetries = numRetries;
            _textReader = streamReader;
        }

        /// <summary>
        /// Send a command to the tcp server
        /// </summary>
        /// <param name="s">A command</param>
        internal async Task WriteLineAsync(string s)
        {
            await _textWriter.WriteLineAsync(s);
            await _textWriter.FlushAsync();
        }

        /// <summary>
        /// Read response data from the tcp server
        /// </summary>
        /// <returns>The string read</returns>
        internal async Task<string> ReadAsync()
        {
            const int blockSize = 512;
            Memory<char> memory = new Memory<char>(new char[blockSize]);
            var count = await _textReader.ReadAsync(memory);
            return new string(memory.Span.Slice(0, count).ToArray());
        }

        /// <summary>
        /// Send a command to the tcp server
        /// </summary>
        /// <param name="commandType">The type of command</param>
        /// <param name="args">The arguments</param>
        protected async Task SendCommandAsync(string commandType, params string[] args)
        {
            string cmd;
            if (args.Any())
                cmd = $"{commandType}:{string.Join(",", args)}";
            else
                cmd = commandType;
            await WriteLineAsync(cmd);
        }

        /// <summary>
        /// Send a command and wait for answere
        /// </summary>
        /// <param name="commandType"></param>
        /// <param name="responseRegex"></param>
        /// <param name="commandArgs"></param>
        /// <returns></returns>
        protected async Task<string> SendAndReceiveAsync(string commandType, string responseRegex, params string[] commandArgs)
        {
            int retryCount = 0;

            while(true)
            {
                await SendCommandAsync(commandType, commandArgs);
                try
                {
                    return await ReceiveAnswerAsync(commandType, responseRegex);
                }
                catch (NiryoOneException e) when (e.Reason == "Robot is busy right now, command ignored." && retryCount < _numRetries)
                { }
                retryCount++;
            }
        }
        private string _stringBuf = "";

        /// <summary>
        /// Receive an answer from the tcp server related to a previously sent command
        /// </summary>
        /// <param name="commandType">The command for which a response is expected</param>
        /// <param name="regex">Optionally, the regular expression that the successful response arguments
        /// are supposed to match</param>
        /// <returns>The data portion of the response</returns>
        internal async Task<string> ReceiveAnswerAsync(string commandType, string regex = "")
        {

            var fullRegex = new Regex($"^[A-Z_]+:(OK{regex}|KO,.*)");
            string s = _stringBuf;
            var sb = new StringBuilder(s);
            while (!fullRegex.IsMatch(s))
            {
                sb.Append(await ReadAsync());
                s = sb.ToString();
            }
            var match = fullRegex.Match(s);
            var result = match.Value.Trim();
            _stringBuf = s.Substring(match.Index + match.Length).TrimStart();

            var colonSplit = result.Split(':', 2);
            var cmd = colonSplit[0];
            if (cmd != commandType)
                throw new NiryoOneException("Wrong command response received.");
            var commaSplit2 = colonSplit[1].Split(',', 2);
            var status = commaSplit2[0];
            if (status != "OK")
                throw new NiryoOneException(commaSplit2[1]);

            if (commaSplit2.Length > 1)
                return commaSplit2[1];
            else
                return string.Empty;
        }

        /// <summary>
        /// Request calibration.
        /// <param name="mode">Whether to request automatic or manual calibration</param>
        /// </summary>
        public async Task Calibrate(CalibrateMode mode)
        {
            await SendAndReceiveAsync("CALIBRATE", null, mode.ToString());
        }

        /// <summary>
        /// Set whether the robot should be in learning mode or not.
        /// <param name="mode">Activate learning mode or not</param>
        /// </summary>
        public async Task SetLearningMode(bool mode)
        {
            await SendAndReceiveAsync("SET_LEARNING_MODE", null, mode.ToString().ToUpper());
        }

        /// <summary>
        /// Move joints to specified configuration.
        /// <param name="joints">The desired destination joint configuration</param>
        /// </summary>
        public async Task MoveJoints(RobotJoints joints)
        {
            await SendAndReceiveAsync("MOVE_JOINTS", null, string.Join(',', joints.Select(x => x.ToString(CultureInfo.InvariantCulture))));
        }

        /// <summary>
        /// Move joints to specified pose.
        /// <param name="pose">The desired destination pose</param>
        /// </summary>
        public async Task MovePose(PoseObject pose)
        {
            await SendAndReceiveAsync("MOVE_POSE", null, string.Join(',', pose.Select(x => x.ToString(CultureInfo.InvariantCulture))));
        }

        /// <summary>
        /// Shift the pose along one axis.
        /// <param name="axis">Which axis to shift</param>
        /// <param name="value">The amount to shift (meters or radians)</param>
        /// </summary>
        public async Task ShiftPose(RobotAxis axis, float value)
        {
            await SendAndReceiveAsync("SHIFT_POSE", null, axis.ToString(), value.ToString(CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// Set the maximum arm velocity.false
        /// <param name="velocity">The maximum velocity in percent of maximum velocity.</param>
        /// </summary>
        public async Task SetArmMaxVelocity(int velocity)
        {
            await SendAndReceiveAsync("SET_ARM_MAX_VELOCITY", null, velocity.ToString());
        }

        /// <summary>
        /// Enable or disable joystick control.false
        /// </summary>
        public async Task EnableJoystick(bool mode)
        {
            await SendAndReceiveAsync("ENABLE_JOYSTICK", null, mode.ToString().ToUpper());
        }

        /// <summary>
        /// Configure a GPIO pin for input or output.
        /// </summary>
        public async Task SetPinMode(RobotPin pin, PinMode mode)
        {
            await SendAndReceiveAsync("SET_PIN_MODE", null, pin.ToString(), mode.ToString());
        }

        /// <summary>
        /// Write to a digital pin configured as output.
        /// </summary>
        public async Task DigitalWrite(RobotPin pin, DigitalState state)
        {
            await SendAndReceiveAsync("DIGITAL_WRITE", null, pin.ToString(), state.ToString());
        }

        /// <summary>
        /// Read from a digital pin configured as input.
        /// </summary>
        public async Task<DigitalState> DigitalRead(RobotPin pin)
        {
            var state = await SendAndReceiveAsync("DIGITAL_READ", ",(0|1|HIGH|LOW)", pin.ToString());
            return (DigitalState)Enum.Parse(typeof(DigitalState), state);
        }

        /// <summary>
        /// Select which tool is connected to the robot.
        /// </summary>
        public async Task ChangeTool(RobotTool tool)
        {
            await SendAndReceiveAsync("CHANGE_TOOL", null, tool.ToString());
        }

        /// <summary>
        /// Open the gripper.
        /// <param name="gripper">Which gripper to open</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task OpenGripper(RobotTool gripper, int speed)
        {
            await SendAndReceiveAsync("OPEN_GRIPPER", null,  gripper.ToString(), speed.ToString());
        }

        /// <summary>
        /// Close the gripper.
        /// <param name="gripper">Which gripper to close</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task CloseGripper(RobotTool gripper, int speed)
        {
            await SendAndReceiveAsync("CLOSE_GRIPPER", null, gripper.ToString(), speed.ToString());
        }

        /// <summary>
        /// Pull air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PullAirVacuumPump(RobotTool vacuumPump)
        {
            await SendAndReceiveAsync("PULL_AIR_VACUUM_PUMP", null, vacuumPump.ToString());
        }

        /// <summary>
        /// Push air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PushAirVacuumPump(RobotTool vacuumPump)
        {
            await SendAndReceiveAsync("PUSH_AIR_VACUUM_PUMP", null, vacuumPump.ToString());
        }

        /// <summary>
        /// Setup the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task SetupElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendAndReceiveAsync("SETUP_ELECTROMAGNET", null, tool.ToString(), pin.ToString());
        }

        /// <summary>
        /// Activate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task ActivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendAndReceiveAsync("ACTIVATE_ELECTROMAGNET", null, tool.ToString(), pin.ToString());
        }

        /// <summary>
        /// Deactivate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task DeactivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendAndReceiveAsync("DEACTIVATE_ELECTROMAGNET", null, tool.ToString(), pin.ToString());
        }

        /// <summary>
        /// Get the current joint configuration.
        /// </summary>
        public async Task<RobotJoints> GetJoints()
        {
            var joints = await SendAndReceiveAsync("GET_JOINTS", "(, *[-0-9.e]+){6}");
            return ParserUtils.ParseRobotJoints(joints);
        }

        /// <summary>
        /// Get the current pose.
        /// </summary>
        public async Task<PoseObject> GetPose()
        {
            var pose = await SendAndReceiveAsync("GET_POSE", "(, *[-0-9.e]+){6}");
            return ParserUtils.ParsePoseObject(pose);
        }

        /// <summary>
        /// Get the current hardware status.
        /// </summary>
        public async Task<HardwareStatus> GetHardwareStatus()
        {
            var status = await SendAndReceiveAsync("GET_HARDWARE_STATUS", @"(, *([^,\[\]()]+|\[[^\[\]()]*\]|\([^\[\]()]*\))){11}");
            return ParserUtils.ParseHardwareStatus(status);
        }

        /// <summary>
        /// Get whether the robot is in learning mode.
        /// </summary>
        public async Task<bool> GetLearningMode()
        {
            var mode = await SendAndReceiveAsync("GET_LEARNING_MODE", ", *(TRUE|FALSE)");
            return bool.Parse(mode);
        }

        /// <summary>
        /// Get the current state of the digital io pins.
        /// </summary>
        public async Task<DigitalPinObject[]> GetDigitalIOState()
        {
            var state = await SendAndReceiveAsync("GET_DIGITAL_IO_STATE", @"(, *\[[^]]*\]){8}");
            var regex = new Regex("\\[[0-9]+, '[^']*', [0-9]+, [0-9+]\\]");
            var matches = regex.Matches(state);
            return matches.Select(m => ParserUtils.ParseDigitalPinObject(m.Value)).ToArray();
        }
    }
}
