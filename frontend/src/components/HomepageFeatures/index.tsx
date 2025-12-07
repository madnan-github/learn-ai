import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'AI-Powered RAG Chatbot',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        <strong>Intelligent Q&A System</strong> built with OpenAI Agents, FastAPI, 
        Qdrant Cloud, and Neon Postgres. Get instant, context-aware answers about 
        Physical AI concepts from the textbook content.
      </>
    ),
  },
  {
    title: 'Personalized Learning Paths',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        <strong>Adaptive content delivery</strong> using better-auth integration. 
        Share your software/hardware background during signup to receive chapters 
        tailored to your expertise level and learning goals.
      </>
    ),
  },
  {
    title: 'Multi-language',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        <strong>One-click Urdu translation</strong> for all educational content. 
        Making Physical AI education accessible to Urdu-speaking students and 
        professionals worldwide.
      </>
    ),
  },
];


function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
