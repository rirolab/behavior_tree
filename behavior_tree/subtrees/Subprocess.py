import datetime
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time

import py_trees


class SUBPROCESS(py_trees.behaviour.Behaviour):
    """
    Run a subprocess until all target output patterns appear.
    """

    def __init__(
        self,
        name,
        command,
        success_text_list,
        timeout,
        failure_text_list=None,
        cwd=None,
        env=None,
        shell=False,
        terminate_timeout=3.0,
        output_encoding="utf-8",
        max_output_chars=None,
        output_log_dir=None,
    ):
        super(SUBPROCESS, self).__init__(name=name)

        # Validate fixed launch settings before the behaviour is ticked.
        if not isinstance(command, (str, list, tuple)):
            raise TypeError(f"{self.name}: command should be str, list, or tuple")
        if isinstance(command, (list, tuple)) and not all(
            isinstance(part, str) for part in command
        ):
            raise TypeError(f"{self.name}: command list should contain only str values")
        if shell and not isinstance(command, str):
            raise TypeError(f"{self.name}: shell=True requires command to be str")
        if failure_text_list is None:
            failure_text_list = []

        # Compile output pattern entries while preserving the existing outer AND semantics.
        success_text_regex_groups, success_text_patterns = self.compile_output_entry_list(
            "success_text_list",
            success_text_list,
            allow_empty=False,
        )
        failure_text_regex_groups, failure_text_patterns = self.compile_output_entry_list(
            "failure_text_list",
            failure_text_list,
            allow_empty=True,
        )

        # Reject shared concrete patterns across terminal states to avoid ambiguous output.
        if set(success_text_patterns).intersection(set(failure_text_patterns)):
            raise ValueError(
                f"{self.name}: success_text_list and failure_text_list should not overlap"
            )
        if not isinstance(timeout, (int, float)) or timeout < 0.0:
            raise ValueError(f"{self.name}: timeout should be a non-negative number")
        if not isinstance(terminate_timeout, (int, float)) or terminate_timeout < 0.0:
            raise ValueError(
                f"{self.name}: terminate_timeout should be a non-negative number"
            )
        if max_output_chars is not None:
            if not isinstance(max_output_chars, int) or max_output_chars <= 0:
                raise ValueError(
                    f"{self.name}: max_output_chars should be a positive int"
                )
            text_patterns = success_text_patterns + failure_text_patterns
            if max_output_chars < max(len(pattern) for pattern in text_patterns):
                raise ValueError(
                    f"{self.name}: max_output_chars should fit the longest output pattern"
                )
        if not isinstance(output_encoding, str) or output_encoding == "":
            raise ValueError(f"{self.name}: output_encoding should be a non-empty str")
        if output_log_dir is not None:
            if not isinstance(output_log_dir, str) or output_log_dir.strip() == "":
                raise ValueError(
                    f"{self.name}: output_log_dir should be a non-empty str"
                )
            if Path(output_log_dir).is_absolute():
                raise ValueError(f"{self.name}: output_log_dir should be relative")

        # Store immutable subprocess configuration.
        self.command = command
        self.success_text_list = list(success_text_list)
        self.success_text_regex_groups = success_text_regex_groups
        self.failure_text_list = list(failure_text_list)
        self.failure_text_regex_groups = failure_text_regex_groups
        self.timeout = float(timeout)
        self.cwd = cwd
        self.env = env
        self.shell = bool(shell)
        self.terminate_timeout = float(terminate_timeout)
        self.output_encoding = output_encoding
        self.max_output_chars = max_output_chars
        self.output_log_dir = output_log_dir

        # Store per-run process state.
        self.process = None
        self.reader_thread = None
        self.output_queue = queue.Queue()
        self.pending_output_line = ""
        self.deadline = None
        self.launch_error = None
        self.matched_success_text_indexes = set()
        self.matched_failure_text_indexes = set()
        self.terminal_output_status = None
        self.output_log_file = None
        self.output_log_path = None

    def initialise(self):
        # Clean up any stale process before starting a new run.
        self.terminate_process()

        # Reset runtime state before launching the process.
        self.process = None
        self.reader_thread = None
        self.output_queue = queue.Queue()
        self.pending_output_line = ""
        self.deadline = time.monotonic() + self.timeout
        self.launch_error = None
        self.matched_success_text_indexes = set()
        self.matched_failure_text_indexes = set()
        self.terminal_output_status = None
        self.output_log_file = None
        self.output_log_path = None

        # Open the optional subprocess output log before launching the command.
        if self.output_log_dir is not None:
            log_dir = Path(self.output_log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", self.name).strip("_")
            self.output_log_path = log_dir / f"{timestamp}_{safe_name}.log"
            self.output_log_file = self.output_log_path.open(
                "a",
                encoding=self.output_encoding,
                errors="replace",
            )

        # Launch the command in its own process group for later cleanup.
        try:
            self.process = subprocess.Popen(
                self.command,
                cwd=self.cwd,
                env=self.env,
                shell=self.shell,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as error:
            self.launch_error = error
            self.feedback_message = f"failed to launch subprocess: {error}"
            return

        # Read output in a background thread so BT ticking never blocks on pipes.
        self.reader_thread = threading.Thread(
            target=self.read_output,
            name=f"{self.name}_output_reader",
            daemon=True,
        )
        self.reader_thread.start()
        self.feedback_message = "subprocess launched"

    def update(self):
        # Surface launch failures as behaviour failures.
        if self.launch_error is not None:
            return py_trees.common.Status.FAILURE
        if self.process is None:
            self.feedback_message = "subprocess was not launched"
            return py_trees.common.Status.FAILURE

        # Fold all available output into complete lines and the active partial line.
        while not self.output_queue.empty():
            chunk = self.output_queue.get_nowait()
            self.pending_output_line += chunk
            lines = self.pending_output_line.splitlines(keepends=True)
            if lines and not lines[-1].endswith(("\n", "\r")):
                self.pending_output_line = lines.pop()
            else:
                self.pending_output_line = ""
            for line in lines:
                self.match_output_patterns(line.rstrip("\r\n"))
            if (
                self.max_output_chars is not None
                and len(self.pending_output_line) > self.max_output_chars
            ):
                self.pending_output_line = self.pending_output_line[
                    -self.max_output_chars:
                ]
            self.match_output_patterns(self.pending_output_line)

        # Report the first terminal output pattern set that completed.
        if self.terminal_output_status == "success":
            self.feedback_message = (
                f"matched output patterns {self.success_text_list}"
            )
            return py_trees.common.Status.SUCCESS
        if self.terminal_output_status == "failure":
            self.feedback_message = (
                f"matched failure output patterns {self.failure_text_list}"
            )
            return py_trees.common.Status.FAILURE

        # Fail if the subprocess exits before every target output pattern appears.
        return_code = self.process.poll()
        if return_code is not None:
            self.feedback_message = (
                f"subprocess exited with code {return_code} "
                f"before matching all output patterns"
            )
            return py_trees.common.Status.FAILURE

        # Fail if every output marker does not arrive within the timeout.
        if time.monotonic() >= self.deadline:
            missing_patterns = [
                pattern
                for index, pattern in enumerate(self.success_text_list)
                if index not in self.matched_success_text_indexes
            ]
            self.feedback_message = (
                f"subprocess timed out before matching output patterns "
                f"{missing_patterns}"
            )
            return py_trees.common.Status.FAILURE

        # Stay running while the process is alive and some patterns are still missing.
        missing_patterns = [
            pattern
            for index, pattern in enumerate(self.success_text_list)
            if index not in self.matched_success_text_indexes
        ]
        self.feedback_message = f"waiting for output patterns {missing_patterns}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Keep ready subprocesses alive until the process group sequence finishes.
        if new_status == py_trees.common.Status.SUCCESS:
            return

        # Stop the subprocess on failure, timeout, launch cleanup, or interruption.
        self.terminate_process()

    def compile_output_entry_list(self, list_name, entry_list, allow_empty):
        # Validate the outer list that keeps AND semantics between entries.
        if not isinstance(entry_list, (list, tuple)):
            raise TypeError(f"{self.name}: {list_name} should be a list or tuple")
        if not allow_empty and not entry_list:
            raise ValueError(f"{self.name}: {list_name} should not be empty")

        # Compile each entry into one OR group and reject duplicate concrete patterns.
        regex_groups = []
        concrete_patterns = []
        seen_patterns = set()
        for entry in entry_list:
            option_patterns = self.read_output_entry_options(list_name, entry)
            for pattern in option_patterns:
                if pattern in seen_patterns:
                    raise ValueError(
                        f"{self.name}: {list_name} should not contain duplicate patterns"
                    )
                seen_patterns.add(pattern)
                concrete_patterns.append(pattern)
            regex_groups.append(
                [
                    self.compile_output_pattern(pattern)
                    for pattern in option_patterns
                ]
            )
        return regex_groups, concrete_patterns

    def read_output_entry_options(self, list_name, entry):
        # Accept one string as a single required output pattern.
        if isinstance(entry, str):
            if entry == "":
                raise ValueError(
                    f"{self.name}: {list_name} should contain non-empty strings"
                )
            return [entry]

        # Accept {"or": [...]} as one required group where any option can match.
        if isinstance(entry, dict):
            if set(entry.keys()) != {"or"}:
                raise ValueError(
                    f"{self.name}: {list_name} dict entries should only use key 'or'"
                )
            options = entry["or"]
            if not isinstance(options, (list, tuple)) or not options:
                raise ValueError(
                    f"{self.name}: {list_name} 'or' value should be a non-empty list or tuple"
                )
            if not all(isinstance(pattern, str) for pattern in options):
                raise TypeError(
                    f"{self.name}: {list_name} 'or' value should contain only str values"
                )
            if not all(pattern for pattern in options):
                raise ValueError(
                    f"{self.name}: {list_name} 'or' value should contain non-empty strings"
                )
            if len(set(options)) != len(options):
                raise ValueError(
                    f"{self.name}: {list_name} 'or' value should not contain duplicates"
                )
            return list(options)

        # Reject every other entry shape instead of guessing caller intent.
        raise TypeError(
            f"{self.name}: {list_name} entries should be str or {{'or': [str, ...]}}"
        )

    @staticmethod
    def compile_output_pattern(pattern):
        # Convert the local wildcard syntax into a regex for one-line matching.
        return re.compile(".*?".join(re.escape(part) for part in pattern.split("*")))

    def match_output_patterns(self, line):
        # Stop matching once either success or failure has already completed.
        if self.terminal_output_status is not None:
            return

        # Collect newly matched output patterns in their observed line order.
        match_events = []
        for index, regex_group in enumerate(self.success_text_regex_groups):
            if index in self.matched_success_text_indexes:
                continue
            earliest_match = None
            for regex in regex_group:
                match = regex.search(line)
                if match and (
                    earliest_match is None
                    or (match.end(), match.start()) < earliest_match
                ):
                    earliest_match = (match.end(), match.start())
            if earliest_match is not None:
                match_events.append((*earliest_match, 1, "success", index))
        for index, regex_group in enumerate(self.failure_text_regex_groups):
            if index in self.matched_failure_text_indexes:
                continue
            earliest_match = None
            for regex in regex_group:
                match = regex.search(line)
                if match and (
                    earliest_match is None
                    or (match.end(), match.start()) < earliest_match
                ):
                    earliest_match = (match.end(), match.start())
            if earliest_match is not None:
                match_events.append((*earliest_match, 0, "failure", index))

        # Apply matches in order so the first completed list decides the result.
        for _, _, _, status_name, index in sorted(match_events):
            if status_name == "success":
                self.matched_success_text_indexes.add(index)
                if len(self.matched_success_text_indexes) == len(self.success_text_list):
                    self.terminal_output_status = "success"
                    return
            else:
                self.matched_failure_text_indexes.add(index)
                if len(self.matched_failure_text_indexes) == len(self.failure_text_list):
                    self.terminal_output_status = "failure"
                    return

    def read_output(self):
        # Forward subprocess output chunks into the BT-owned queue.
        try:
            while self.process is not None and self.process.stdout is not None:
                chunk = os.read(self.process.stdout.fileno(), 4096)
                if chunk == b"":
                    break
                decoded_chunk = chunk.decode(self.output_encoding, errors="replace")
                self.write_output_log(decoded_chunk)
                self.output_queue.put(decoded_chunk)
        except OSError:
            return

    def write_output_log(self, text):
        # Append decoded process output to the optional log file.
        if self.output_log_file is None:
            return
        self.output_log_file.write(text)
        self.output_log_file.flush()

    def close_output_log(self):
        # Close the optional log file after the reader thread finishes.
        if self.output_log_file is None:
            return
        self.output_log_file.close()
        self.output_log_file = None

    def terminate_process(self):
        # Ignore process cleanup when it never launched or already exited.
        if self.process is None or self.process.poll() is not None:
            if self.reader_thread is not None and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
            self.close_output_log()
            return

        # Ask the whole subprocess group to terminate first.
        process_group_missing = False
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            process_group_missing = True
        except PermissionError:
            self.process.terminate()
        if process_group_missing:
            if self.reader_thread is not None and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
            self.close_output_log()
            return

        # Escalate to SIGKILL if graceful termination does not finish in time.
        try:
            self.process.wait(timeout=self.terminate_timeout)
        except subprocess.TimeoutExpired:
            process_group_missing = False
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                process_group_missing = True
            except PermissionError:
                self.process.kill()
            if process_group_missing:
                if self.reader_thread is not None and self.reader_thread.is_alive():
                    self.reader_thread.join(timeout=1.0)
                self.close_output_log()
                return
            try:
                self.process.wait(timeout=self.terminate_timeout)
            except subprocess.TimeoutExpired:
                self.feedback_message = "subprocess did not exit after SIGKILL"

        # Finish any remaining pipe reads and close the optional output log.
        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        self.close_output_log()


class PROCESS_GROUP_SEQ(py_trees.composites.Sequence):
    """
    Sequence that terminates all nested SUBPROCESS instances when it stops.
    """

    def __init__(self, name, children=None, memory=True):
        super(PROCESS_GROUP_SEQ, self).__init__(name=name, memory=memory)

        # Attach optional children using the same style as local composite wrappers.
        if children:
            self.add_children(children)

    def terminate(self, new_status):
        # Clean up subprocesses after success, failure, or external interruption.
        for node in self.iterate():
            if isinstance(node, SUBPROCESS):
                node.terminate_process()
