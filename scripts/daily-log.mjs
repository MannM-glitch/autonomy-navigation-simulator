import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import process from 'node:process';

const root = process.cwd();
const missionsPath = path.join(root, 'data', 'missions.json');
const progressPath = path.join(root, 'data', 'progress.json');
const logDir = path.join(root, 'engineering-log');

const today = new Date().toISOString().slice(0, 10);
const missions = JSON.parse(await readFile(missionsPath, 'utf8'));
const progress = JSON.parse(await readFile(progressPath, 'utf8'));
const missionIndex = progress.currentMissionIndex % missions.length;
const mission = missions[missionIndex];
const logPath = path.join(logDir, `${today}.md`);

await mkdir(logDir, { recursive: true });

const content = `# ${today}: ${mission.title}

## Robotics skill

${mission.skill}

## Concept

${mission.concept}

## Experiment

${mission.lab}

## Evidence to add

${mission.proof}

## Notes

- Prediction:
- Observation:
- Finding:
- What I will improve next:
`;

await writeFile(logPath, content, 'utf8');

const history = Array.isArray(progress.history) ? progress.history : [];
const nextHistory = [
  ...history.filter((entry) => entry.date !== today),
  {
    date: today,
    missionDay: mission.day,
    title: mission.title,
    skill: mission.skill
  }
].sort((a, b) => a.date.localeCompare(b.date));

await writeFile(
  progressPath,
  `${JSON.stringify(
    {
      currentMissionIndex: missionIndex + 1,
      lastGeneratedAt: new Date().toISOString(),
      history: nextHistory
    },
    null,
    2
  )}\n`,
  'utf8'
);

console.log(`Created ${path.relative(root, logPath)} for ${mission.skill}.`);
