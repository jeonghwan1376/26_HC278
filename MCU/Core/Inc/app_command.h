#ifndef INC_APP_COMMAND_H_
#define INC_APP_COMMAND_H_

#ifdef __cplusplus
extern "C" {
#endif

/* master에서 수신한 한 줄 명령을 각 기능 모듈로 전달 */
void AppCommand_ProcessLine(const char *line);

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_COMMAND_H_ */
